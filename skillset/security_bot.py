# Prepare for Python3 conversion
from __future__ import absolute_import
from __future__ import print_function

# Math helpers
from math import radians
import numpy as np
from shared.util.time_manager.time_manager import DownSampler

# The base class for all Skills
from vehicle.skills.base_skill import Skill

# UiElements
from vehicle.skills.util.ui import UiButton
from vehicle.skills.util.ui import UiSlider


def get_closest_person(subject_api, reference_position, min_radius=1e6):
    closest = None
    for track in subject_api.get_all_tracks():
        if track.classification != 'PERSON':
            continue
        delta = np.linalg.norm(reference_position - track.position)
        if delta < min_radius:
            min_radius = delta
            closest = track
    return closest


class SecurityBot(Skill):
    """
    Visually scan the area, counting people.

    If someone gets too close the vehicle, fly above them.
    """

    # These are custom settings that will appear during flight as adjustable UI in the Skydio app.
    USER_SETTINGS = (
        UiSlider(
            identifier="search_radius",
            label="Search Radius",
            detail="Start following anyone that comes within this range of the home point.",
            min_value=1,
            max_value=30,
            value=5,
            units="m",
        ),
        UiSlider(
            identifier="scan_rate",
            label="Scanning Rate",
            detail="How quickly the vehicle should rotate while looking for targets.",
            min_value=1,
            max_value=30,
            value=10,
            units="deg/s",
        ),
        UiSlider(
            identifier="follow_speed",
            label="Follow Speed",
            detail='How quickly the vehicle should move when following a subject.',
            min_value=1,
            max_value=8,
            value=4,
            units="m/s",
        ),
    )

    def __init__(self):
        super(SecurityBot, self).__init__()
        self.home_point = None
        self.running = True
        self.following = False
        self.status_downsampler = DownSampler(1.0)

    def button_pressed(self, api, button_id, source):  # pylint: disable=unused-argument
        """ Called by the sdk whenever the user presses a button """
        print("user pressed {}".format(button_id))
        if button_id == 'set_point':
            self.running = True
            self.home_point = api.vehicle.get_position()

        elif button_id == 'stop':
            self.running = False
            self.following = False

    def get_onscreen_controls(self, api):
        """ Add buttons and titles to the app based on current skill state. """
        controls = dict()

        if self.running:
            # Show a title based on detected objects
            num_detections = len(api.subject.get_all_tracks())
            closest_object = get_closest_person(api.subject, self.home_point)
            if closest_object:
                distance = np.linalg.norm(closest_object.position - self.home_point)
            else:
                distance = -1

            if not self.following:
                controls['title'] = 'Searching'
                controls['detail'] = \
                    'Detections: {}\nClosest: {:.1f}m'.format(num_detections, distance)
            else:
                controls['title'] = 'Following'
                controls['detail'] = ''

            # Show the red STOP button
            controls['show_stop'] = True

            # Hide the manual controls and buttons
            controls['height_slider_enabled'] = False
            controls['buttons'] = []

        else:
            controls['title'] = 'Set Home Point'
            controls['detail'] = ''

            # Enable manual controls and a Start Button
            controls['height_slider_enabled'] = True
            controls['buttons'] = [UiButton(identifier='set_point', label='Set')]

            # Hide the stop button
            controls['show_stop'] = False

        return controls

    def update(self, api):
        if not self.running:
            api.subject.request_no_subject(api.utime)
            # Re-enable manual control
            api.phone.enable_movement_commands()
            return

        # Update the display periodically so we can see stats change.
        if self.status_downsampler.ready(api.utime):
            self.set_needs_layout()

        # Disable manual control during autonomous motion.
        api.phone.disable_movement_commands()

        # Tripod style controls
        api.focus.apply_tripod_presets()

        if self.home_point is None:
            self.home_point = api.vehicle.get_position()

        # Find the closest track
        search_radius = self.get_value_for_user_setting('search_radius')
        closest_object = get_closest_person(api.subject, self.home_point, min_radius=search_radius)

        # Move to track, if any are close enough
        if closest_object is not None:
            if not self.following:
                self.following = True
                print("following!")
                self.set_needs_layout()
            api.subject.select_track(api.utime, closest_object.track_id)
            api.movement.set_desired_pos_nav(closest_object.position + np.array([0, 0, 3.0]))
            follow_speed = self.get_value_for_user_setting('follow_speed')
            api.movement.set_max_speed(follow_speed)

        else:
            if self.following:
                self.following = False
                print("stop following!")
                self.set_needs_layout()
            # Otherwise, spin around and search
            api.subject.cancel_if_following(api.utime)
            scan_rate = radians(self.get_value_for_user_setting('scan_rate'))
            api.movement.set_heading_rate(scan_rate)
            api.movement.set_gimbal_pitch(0)

            # Move back to home
            api.movement.set_desired_pos_nav(self.home_point)
            api.movement.set_max_speed(2.0)
