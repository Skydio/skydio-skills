# Prepare for Python3 conversion
from __future__ import absolute_import
from __future__ import print_function
import enum
import math
import numpy as np
from numpy.linalg import norm
import random
import string

# Import from the Skydio2 skills sdk
import shared.util.error_reporter.error_reporter as er
import shared.util.time_manager.time_manager as tm
from vehicle.skills.util import core
from vehicle.skills.base_skill import Skill
from vehicle.skills.util import LowPassFilter
from vehicle.skills.util.ui import UiButton
from vehicle.skills.util.ui import UiSlider

# pylint: disable=too-many-statements


class PartyMode(Skill):
    """ Searches in an area, locking onto people and staying on them if they are moving."""

    USER_SETTINGS = (
        UiSlider(
            identifier="radius",
            label="Max Distance",
            detail="The maximum distance from the home point",
            min_value=15,
            max_value=2000,
            value=100,
            units="m",
        ),
        UiSlider(
            identifier="duration",
            label="Max Tracking Time",
            detail="The maximum time the vehicle will track a single person",
            min_value=10.0,
            max_value=200.0,
            value=30.0,
            units="s",
        ),
        UiSlider(
            identifier="speed",
            label="Return Speed",
            detail="The speed the vehicle should fly when returning to home.",
            min_value=1.0,
            max_value=10.0,
            value=5.0,
            units="m/s",
        ),
    )

    # Amount of time to wait after lock
    LOCK_WAIT_TIME = 2.0  # [s]
    # Amount of time to wait after unlock
    UNLOCK_WAIT_TIME = 1.0  # [s]
    # Maximum subject distance to lock on
    MAX_LOCK_DISTANCE = 8.0  # [m]
    # When sending the vehicle to return, project this far before sending the desired position
    MAX_DESIRED_POSITION_DISTANCE = 5.0  # [m]
    # When in search mode, how fast to yaw
    SEARCH_HEADING_RATE = math.radians(12)  # [rad]
    # When in search mode, the pitch angle of the gimbal
    SEARCH_GIMBAL_PITCH = math.radians(17)  # [rad]
    # The minimum height that the vehicle will go relative to the subject.
    MIN_HEIGHT = 2.0  # [m]
    # Minimum time to wait before re-locking onto a previously locked track
    MIN_REPEAT_TRACK_TIME = 20.0  # [s]
    # Maximum amount of time to follow a track that doesn't move
    MAX_TRACK_STILL_TIME = 8.0  # [s]
    # A track that exceeds this speed is defined as moving.
    MOVING_CUTOFF_SPEED = 1.0  # [m/s]

    STATE = enum.Enum('STATE', 'SEARCH LOCKING LOCKED UNLOCKING RETURN PAUSE')

    def __init__(self):
        super(PartyMode, self).__init__()

        # The point that we're anchored at
        self.nav_t_anchor = None

        # Maps track ids to the last utime that we were tracking them, to not repeat tracks too
        # soon.
        self.trackid_last_utime_dict = dict()

        # Multiply this by the direction of the rate to change up the search strategy
        self.rate_direction_mult = 1

        self.state = None

        # utime of the last state transition
        self.last_transition_utime = None

        # has the current subject track exceeded the min cutoff speed for movement?
        self.subject_has_moved = False
        self.subject_speed_filter = LowPassFilter(0.5)

        self.dynamic_aggressiveness = 0.75
        self.publish_downsampler = tm.DownSampler(0.5)

    def find_and_lock_subject(self, api):
        # Returns true if we are currently tracking a subject
        # or if not, if we were able to find a good candidate to start tracking.

        # A good candidate is a track that we haven't recently finished tracking, is locked,
        # and is close enough.
        vehicle_position = api.vehicle.get_position()
        motion_state = api.subject.get_motion_state()
        tracker_state = motion_state.tracker_state

        selected_track = api.subject.get_subject_track()
        if selected_track is not None:
            self.trackid_last_utime_dict[selected_track.track_id] = motion_state.utime
            return True

        locked_tracks = []
        for track in tracker_state.tracks:
            if track.classification.name != 'PERSON':
                # skip non-person tracks
                continue
            track_position = core.msg_to_ndarray(track.nav_frame.trans.position)
            distance = norm(track_position - vehicle_position)
            distance_from_start = norm(track_position - self.nav_t_anchor)

            # Check that this trackid hasn't been tracked in some amount of time
            been_long_enough = True
            if track.track_id in self.trackid_last_utime_dict:
                last_time = self.trackid_last_utime_dict[track.track_id]
                time_since_last = tm.utime_to_seconds(motion_state.utime - last_time)
                if time_since_last < self.MIN_REPEAT_TRACK_TIME:
                    been_long_enough = False

            er.REPORT_QUIET(
                "Track[id: {}, locked: {}, distance: {}, from_start: {} been_long_enough: {}]",
                track.track_id, track.is_locked, distance, distance_from_start, been_long_enough)

            if track.is_locked and (distance < self.MAX_LOCK_DISTANCE) and been_long_enough and (
                    distance_from_start < self.get_value_for_user_setting('radius')):
                locked_tracks.append(track)

        if locked_tracks:
            chosen_track = random.choice(locked_tracks)
            er.REPORT_QUIET("Found {} locked tracks, chose track {}", len(locked_tracks),
                            chosen_track.track_id)

            self.trackid_last_utime_dict[chosen_track.track_id] = motion_state.utime
            api.subject.select_track(motion_state.utime, chosen_track.track_id)
            return True

        return False

    def transition(self, utime, new_state):
        if new_state is self.STATE.SEARCH:
            # Switch direction each time we start a search.
            self.rate_direction_mult *= -1.0

        if new_state is self.STATE.LOCKED:
            self.subject_has_moved = False
            self.subject_speed_filter.reset()

        er.REPORT_QUIET("Transition: {} -> {}", self.state, new_state)
        self.state = new_state
        self.last_transition_utime = utime
        self.set_needs_layout()

    def get_time_since_transition(self, utime):
        return tm.utime_to_seconds(utime - self.last_transition_utime)

    def get_return_desired_point_nav(self, vehicle_position):
        to_home_vector = self.nav_t_anchor - vehicle_position
        distance_to_home = norm(to_home_vector)
        if distance_to_home > self.MAX_DESIRED_POSITION_DISTANCE:
            to_home_vector = (to_home_vector / distance_to_home) * \
                self.MAX_DESIRED_POSITION_DISTANCE
        return vehicle_position + to_home_vector

    def get_onscreen_controls(self, api):
        if self.state == self.STATE.PAUSE:
            title = 'Paused'
            show_stop = False
            buttons = [UiButton('resume', 'Resume')]
            detail = ''
        elif self.state == self.STATE.LOCKED:
            title = 'Following'
            if not self.subject_has_moved:
                detail = 'waiting for movement'
            else:
                locked_time = self.get_time_since_transition(api.utime)
                time_left = int(self.get_value_for_user_setting('duration') - locked_time)
                detail = 'time left: {}s'.format(time_left)
            buttons = []
            show_stop = True
        else:
            title = string.capitalize(self.state.name)
            detail = ''
            buttons = []
            show_stop = True

        controls = {}
        controls['arrows_enabled'] = not show_stop
        controls['buttons'] = buttons
        controls['height_slider_enabled'] = not show_stop
        controls['show_stop'] = show_stop
        controls['title'] = title
        controls['detail'] = detail
        return controls

    def button_pressed(self, api, button_id, source):  # pylint: disable=unused-argument
        if button_id == 'resume':
            self.transition(api.utime, self.STATE.SEARCH)
        elif button_id == 'stop':
            api.subject.cancel_if_following(api.utime)
            self.transition(api.utime, self.STATE.PAUSE)

    def update(self, api):
        # pylint:disable=too-many-return-statements, too-many-branches

        # Set the ceiling height for safety
        api.focus.settings.min_relative_height_enabled = True
        api.focus.settings.min_relative_height = self.MIN_HEIGHT

        # Lower dynamic aggressiveness for safety. Maybe keep?
        api.planner.settings.dynamics_aggressiveness = self.dynamic_aggressiveness

        if self.publish_downsampler.ready(api.utime):
            self.set_needs_layout()

        if self.state is None:
            return self.transition(api.utime, self.STATE.SEARCH)

        if self.nav_t_anchor is None:
            self.nav_t_anchor = api.vehicle.get_position()
            return

        vehicle_position = api.vehicle.get_position()
        if vehicle_position is None:
            return

        motion_state = api.subject.get_motion_state()
        if motion_state is None:
            return

        # Force going into unlock mode if we're too far from the leash origin
        distance_from_start = norm(vehicle_position - self.nav_t_anchor)

        # The primary state machine logic
        if self.state is self.STATE.SEARCH:
            api.phone.disable_movement_commands()

            # Transitions
            if distance_from_start > self.get_value_for_user_setting('radius'):
                return self.transition(api.utime, self.STATE.RETURN)

            # Try to lock the subject
            if self.find_and_lock_subject(api):
                return self.transition(api.utime, self.STATE.LOCKING)

            # Movement
            api.movement.set_heading_rate(self.SEARCH_HEADING_RATE * self.rate_direction_mult, 1.0)
            api.movement.set_gimbal_pitch(self.SEARCH_GIMBAL_PITCH, 0.5)
            return
        elif self.state is self.STATE.LOCKING:
            api.phone.disable_movement_commands()

            # Transitions
            if self.get_time_since_transition(api.utime) > self.LOCK_WAIT_TIME:
                if motion_state.subject_locked:
                    return self.transition(api.utime, self.STATE.LOCKED)
                else:
                    return self.transition(api.utime, self.STATE.SEARCH)

            return
        elif self.state is self.STATE.LOCKED:
            api.phone.disable_movement_commands()

            api.focus.set_azimuth_rate(0.5)

            locked_time = self.get_time_since_transition(api.utime)

            subject_velocity = api.subject.get_velocity(default_if_none=np.zeros(3))
            self.subject_speed_filter.step(norm(subject_velocity), api.utime)
            subject_speed_filtered = self.subject_speed_filter.get()
            if (not self.subject_has_moved) and \
                    (subject_speed_filtered > self.MOVING_CUTOFF_SPEED):
                self.subject_has_moved = True

            er.REPORT_QUIET("Tracking: {}s, Has moved: {}, Filtered Speed: {}",
                            locked_time, self.subject_has_moved, subject_speed_filtered)

            # Transitions
            if not motion_state.subject_locked:
                return self.transition(api.utime, self.STATE.UNLOCKING)

            if distance_from_start > self.get_value_for_user_setting('radius'):
                return self.transition(api.utime, self.STATE.UNLOCKING)

            # If the subject hasn't moved, cancel tracking
            if (not self.subject_has_moved) and (locked_time > self.MAX_TRACK_STILL_TIME):
                return self.transition(api.utime, self.STATE.UNLOCKING)

            # If it's been too long, cancel tracking
            if locked_time > self.get_value_for_user_setting('duration'):
                return self.transition(api.utime, self.STATE.UNLOCKING)

            return
        elif self.state is self.STATE.UNLOCKING:
            api.phone.disable_movement_commands()
            if motion_state.subject_locked:
                api.subject.request_no_subject(api.utime)

            # Transitions
            if (self.get_time_since_transition(api.utime) > self.UNLOCK_WAIT_TIME) and \
                    (not motion_state.subject_locked):
                return self.transition(api.utime, self.STATE.SEARCH)

            return
        elif self.state is self.STATE.RETURN:
            api.phone.disable_movement_commands()

            # Transitions
            if distance_from_start < 1.0:
                return self.transition(api.utime, self.STATE.SEARCH)

            # Try to lock the subject on the way back
            if self.find_and_lock_subject(api):
                return self.transition(api.utime, self.STATE.LOCKING)

            # Movement
            goto_point = self.get_return_desired_point_nav(vehicle_position)
            api.movement.set_desired_pos_nav(goto_point, 0.5)
            return_speed = self.get_value_for_user_setting('speed')
            api.movement.set_max_speed(return_speed)
            api.movement.set_heading_rate(self.SEARCH_HEADING_RATE * self.rate_direction_mult, 1.0)
            api.movement.set_gimbal_pitch(self.SEARCH_GIMBAL_PITCH, 0.5)
            return
        elif self.state is self.STATE.PAUSE:
            api.phone.enable_movement_commands()
