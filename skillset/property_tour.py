# Prepare for Python3 conversion
from __future__ import absolute_import
from __future__ import print_function
import enum
import math
import numpy as np

# Import from the Skydio2 skills sdk
from shared.util.error_reporter import error_reporter as er
from shared.util.time_manager import time_manager as tm
from vehicle.skills.base_skill import Skill
from vehicle.skills.util import core
from vehicle.skills.util.motions import CableMotion
from vehicle.skills.util.motions import LookatMotion
from vehicle.skills.util.motions import OrbitMotion
from vehicle.skills.util.transform import Rot3
from vehicle.skills.util.transform import Transform
from vehicle.skills.util.ui import UiButton
from vehicle.skills.util.ui import UiSlider


class TourState(enum.Enum):
    # Wait for the user to start the tour
    SETUP = 0

    # Execute the tour
    GOTO = 1

    # Tapped STOP, quit the tour
    STOP = 5


class PropertyTour(Skill):
    """
    Create a simple tour based on the front door of a house
    and some basic size settings.
    """

    USER_SETTINGS = (
        UiSlider(
            identifier='speed',
            label='Speed',
            detail="Maximum speed during motion.",
            units='m/s',
            min_value=0.1,
            max_value=8.0,
            value=6.0,
        ),
        UiSlider(
            identifier='width',
            label='Width',
            detail="The width of the house, assuming drone is centered.",
            units='m',
            min_value=5,
            max_value=25,
            value=10,
        ),
        UiSlider(
            identifier='depth',
            label='Depth',
            detail="The depth of the house, assuming drone is at one end.",
            units='m',
            min_value=0,
            max_value=50,
            value=15,
        ),
        UiSlider(
            identifier='height',
            label='Height',
            detail="Max altitude to fly.",
            units='m',
            min_value=5,
            max_value=30,
            value=15,
        ),
        UiSlider(
            identifier='radius',
            label='Radius',
            detail="Orbit radius",
            units='m',
            min_value=5,
            max_value=40,
            value=15,
        ),
    )

    def __init__(self):
        super(PropertyTour, self).__init__()
        self.state = TourState.SETUP
        self.has_subject = False
        self.publish_downsampler = tm.DownSampler(0.5)

        self.params = core.AttrDict()
        self.params.speed = 4.0
        self.params.min_turn_rate = 1  # [rad/update]
        self.params.max_turn_rate = 20  # [rad/update]
        self.params.distance_margin = 0.5  # [m]
        self.params.angle_margin = math.radians(0.75)  # [rad]
        self.params.giveup_utime = tm.seconds_to_utime(3)  # [s]
        self.params.giveup_speed = 0.1  # [m/s]
        self.params.route_around_obstacles = True

        self.motions = []
        self.motion_index = None

        # AR cache
        self.ar_frames = []  # list of Transforms for each cube in the frame(s)

    def allow_manual_control(self):
        return self.state in (TourState.STOP, TourState.SETUP)

    def button_pressed(self, api, button_id_pressed, source):  # pylint: disable=unused-argument
        if button_id_pressed == 'go':
            self.state = TourState.GOTO

            # Remember the current location.
            nav_T_cam = api.vehicle.get_camera_trans()

            yaw = nav_T_cam.get_euler_angles()[2]
            nav_T_cam_flat = Transform(
                rotation=Rot3.Ypr(yaw, 0, 0),
                translation=nav_T_cam.translation(),
            )

            width = self.get_value_for_user_setting('width')
            depth = self.get_value_for_user_setting('depth')
            height = self.get_value_for_user_setting('height')
            radius = self.get_value_for_user_setting('radius')

            # The first point is the home point.
            nav_T_start = nav_T_cam.copy()

            # Fly backward
            nav_T_back = Transform(
                rotation=nav_T_cam_flat.rotation(),
                translation=nav_T_cam_flat * np.array([-radius / 2, 0, 0]),
            )

            # Fly Up
            nav_T_top = Transform(
                rotation=Rot3.Ypr(yaw, math.radians(30), 0),
                translation=nav_T_cam_flat * np.array([-radius, 0, height])
            )

            # Fly over the house
            nav_T_over = Transform(
                rotation=nav_T_cam_flat.rotation(),
                translation=nav_T_cam_flat * np.array([depth, 0, height / 2]),
            )

            # Crane down
            nav_T_down = Transform(
                rotation=Rot3.Ypr(yaw, math.radians(88), 0),
                translation=nav_T_cam_flat * np.array([-radius / 4, 0, height]),
            )

            # Go left
            nav_T_left = Transform(
                rotation=nav_T_cam_flat.rotation(),
                translation=nav_T_cam_flat * np.array([-5, width / 2, 0]),
            )

            # Go Right
            nav_T_right = Transform(
                rotation=nav_T_cam_flat.rotation(),
                translation=nav_T_cam_flat * np.array([-5, -width / 2, 0]),
            )

            # The center of the house
            nav_T_lookat = Transform(
                rotation=nav_T_cam_flat.rotation(),
                translation=nav_T_cam_flat * np.array([depth / 2, 0, 0]),
            )

            self.motions = [
                # Cable backward as far as possible
                CableMotion(nav_T_start, nav_T_top, self.params),

                # Move while looking at the house
                LookatMotion(
                    start_point=nav_T_top * np.array([0, width, 0]),
                    end_point=nav_T_top * np.array([0, -width, 0]),
                    lookat_point=nav_T_lookat.translation(),
                    params=self.params,
                ),

                # Orbit around the property
                OrbitMotion(nav_T_lookat.translation(), radius, height, 1, self.params),

                # Fly down to the ground
                CableMotion(nav_T_top, nav_T_back, self.params),

                # Fly over the roof
                CableMotion(nav_T_back, nav_T_over, self.params),

                # Fly back and look down
                CableMotion(nav_T_over, nav_T_down, self.params),

                # Crane down to front door
                CableMotion(nav_T_down, nav_T_start, self.params),

                # Fly left
                CableMotion(nav_T_start, nav_T_left, self.params),

                # Fly right
                CableMotion(nav_T_left, nav_T_right, self.params),

                # Back to start
                CableMotion(nav_T_right, nav_T_start, self.params),

                # TODO:
                # Various panoramas in different spots
            ]
            self.motion_index = 0
            er.REPORT_STATUS("go pressed")

        elif button_id_pressed == 'skip':
            er.REPORT_STATUS("skip")
            self.motion_index += 1
            if self.motion_index >= len(self.motions):
                self.motion_index = 0

        if button_id_pressed == 'goto_cable':
            self.state = TourState.GOTO

        if button_id_pressed == 'stop':
            er.REPORT_STATUS('stopped')
            api.subject.cancel_subject_tracking(api.utime)
            self.state = TourState.STOP

    def get_motion(self):
        if self.motion_index is None or self.motion_index >= len(self.motions):
            return None
        return self.motions[self.motion_index]

    def update(self, api):
        # Stop tracking
        api.subject.cancel_subject_tracking(api.utime)

        # Update the speed dynamically, in case the user changes it
        self.params.speed = self.get_value_for_user_setting('speed')

        if self.allow_manual_control():
            api.phone.enable_movement_commands()
        else:
            api.phone.disable_movement_commands()

        if api.planner.is_landing():
            er.REPORT_STATUS("Exiting due to planner landing")
            api.skills.request_skill(api.utime, 'Basic')

        self.has_subject = api.subject.is_following_subject(api.utime)

        # Execute the current motion.
        if api.vehicle.get_pose():
            if self.state == TourState.GOTO:
                motion = self.get_motion()
                if motion:
                    motion.update(api)
                    if motion.done:
                        motion.reset(api)
                        er.REPORT_STATUS("motion done, index ++ {}", self.motion_index)
                        self.motion_index += 1
                    else:
                        er.REPORT_STATUS("motion in progress")
                else:
                    er.REPORT_STATUS("No more motions")
                    self.state = TourState.STOP
            else:
                er.REPORT_QUIET("Not active")
        else:
            er.REPORT_QUIET("Vehicle not ready")

        # Update the ui periodically
        if self.publish_downsampler.ready(api.utime):
            self.set_needs_layout()

    def get_onscreen_controls(self, api):
        title = ''
        detail_text = ''
        buttons = []
        show_stop = True
        controls_enabled = False
        targets_enabled = False
        show_slider = False
        promoted_control_id = ''
        battery_low_land = api.health_monitor.is_battery_critically_low()

        if battery_low_land:
            title = 'Low Battery'
            detail_text = 'Tour disabled'
            controls_enabled = True

        elif self.state == TourState.SETUP:
            title = 'Press Go to Begin Tour'
            detail_text = 'Drone will automatically film the area.'
            controls_enabled = True
            show_stop = False
            targets_enabled = False
            buttons.append(UiButton('go', 'Go', style='PRIMARY'))
            promoted_control_id = 'go'

        elif self.state == TourState.GOTO:
            title = 'Tour in Progress'
            detail_text = "Step {}".format(self.motion_index)
            show_slider = True
            targets_enabled = False
            buttons.append(UiButton('skip', label='Skip'))

        elif self.state == TourState.STOP:
            title = 'Press Go to Start New Tour'
            controls_enabled = True
            show_stop = False
            targets_enabled = False
            buttons.append(UiButton('go', 'Go', style='PRIMARY'))
            promoted_control_id = 'go'

        else:
            er.REPORT_WARNING('Unknown state "{}"'.format(self.state))

        return dict(
            title=title,
            detail=detail_text,
            arrows_enabled=controls_enabled,
            height_slider_enabled=controls_enabled,
            zoom_slider_enabled=controls_enabled and self.has_subject,
            steering_enabled=controls_enabled and not self.has_subject,
            tap_targets_enabled=targets_enabled,
            double_tap_enabled=controls_enabled,
            drag_enabled=controls_enabled,
            show_stop=show_stop,
            promoted_control='speed' if show_slider else promoted_control_id,
            buttons=buttons,
        )
