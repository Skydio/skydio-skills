# Prepare for Python3 conversion
from __future__ import absolute_import
from __future__ import print_function

# Math helpers
from math import pi, cos, sin
import numpy as np
from shared.util.time_manager import time_manager as tm
from vehicle.skills.util import core

# The base class for all Skydio2 Skills
from vehicle.skills.base_skill import Skill

# UiElements
from vehicle.skills.util.ui import UiButton
from vehicle.skills.util.ui import UiRadioGroup
from vehicle.skills.util.ui import UiRadioOption
from vehicle.skills.util.ui import UiSlider

# Augmented Reality Support
from vehicle.skills.util.transform import Rot3, Transform
from vehicle.skills.util.ar import Prism


M_2PI = 2 * pi


class PolygonPath(Skill):
    """
    Fly in the shape of a polygon with a user-defined number of sides.
    """

    # These are custom settings that will appear during flight as adjustable UI in the Skydio app.
    USER_SETTINGS = (
        UiSlider(
            identifier="num_sides",
            label="Sides",
            detail="The number of sides for this polygon.",
            min_value=3,
            max_value=8,
            value=4,
            units="",
        ),
        UiSlider(
            identifier="radius",
            label="Radius",
            detail="The distance from the center of the polygon to one of its vertices.",
            min_value=1,
            max_value=20,
            value=4,
            units="m",
        ),
        UiRadioGroup(
            identifier='direction',
            label='Direction',
            detail='',
            options=[
                UiRadioOption(
                    identifier='clockwise',
                    label='Clockwise',
                    detail='',
                ),
                UiRadioOption(
                    identifier='counter_clockwise',
                    label='Counter-Clockwise',
                    detail='',
                ),
            ],
            selected_option=1,
        ),
    )

    def __init__(self):
        """ Constructor for the Skill. Called whenever the user switches the active skill. """
        super(PolygonPath, self).__init__()
        # whether we are executing the polygon motion, or waiting for user control input.
        self.running = False

        # the position of the center of the polygon in the nav frame
        self.center = None

        # the current vertex we are targeting
        self.index = 0

        # the 3d position of that vertex
        self.desired_position = None

        # the system time when we switched to that vertex
        self.last_change_utime = -1

        # create a downsampler so we can update AR at most once a second
        self.publish_downsampler = tm.DownSampler(1.0)

    def button_pressed(self, api, button_id, source):  # pylint: disable=unused-argument
        """ Called by the sdk whenever the user presses a button """
        print("user pressed {}".format(button_id))
        if button_id == 'start':
            self.running = True
        elif button_id == 'stop':
            self.running = False

    def setting_changed(self, api, setting_id):
        """ Called by the sdk whenever the user adjusts a setting """
        self.desired_position = None
        if setting_id == 'num_sides':
            self.index = 0

    def get_onscreen_controls(self, api):
        """ Add buttons and titles to the skydio app based on current skill state. """
        num_sides = int(self.get_value_for_user_setting('num_sides'))
        controls = dict()
        if self.running:
            # Show a title based on the current index.
            current = int(self.index + 1)
            controls['title'] = 'Vertex {}/{}'.format(current, num_sides)

            # Show the red STOP button
            controls['show_stop'] = True

            # Hide the manual controls and buttons
            controls['height_slider_enabled'] = False
            controls['buttons'] = []

        else:
            # Confirm the number of sides in the polygon with a title
            controls['title'] = '{}-Sided Polygon'.format(num_sides)

            # Enable manual controls and a Start Button
            controls['height_slider_enabled'] = True
            controls['buttons'] = [UiButton(identifier='start', label='Start')]

            # Hide the stop button
            controls['show_stop'] = False

        return controls

    def update_ar_scene(self, api):
        """Draw prisms in the shape of the polygon."""
        api.scene.clear_all_objects()

        num_sides = float(self.get_value_for_user_setting('num_sides'))
        radius = self.get_value_for_user_setting('radius')
        adjust_angle = pi - (num_sides - 2) * pi / num_sides / 2

        for side in range(int(num_sides)):
            desired_angle = side / num_sides * M_2PI
            side_rot = Rot3.Ypr(adjust_angle + desired_angle, 0, 0)
            vertex_offset = radius * np.array([cos(desired_angle), sin(desired_angle), 0])
            vertex_pos = self.center + vertex_offset
            nav_T_vertex = Transform(side_rot, vertex_pos)
            size = 2 * sin(pi / num_sides) * radius
            prism_pos = nav_T_vertex * np.array([size / 2, 0, -1.3])
            p = Prism(nav_T_center=Transform(side_rot, prism_pos),
                      size=np.array([size, .2, .2]))
            api.scene.add_prism(p)

    def update(self, api):
        """ Called by the sdk multiple times a second. """
        if not self.running:
            # Re-enable manual control
            api.phone.enable_movement_commands()
            self.center = None
            return

        # Stop tracking any subjects
        api.subject.cancel_subject_tracking(api.utime)

        # Disable manual control during autonomous motion.
        api.phone.disable_movement_commands()

        # Get latest setting values
        num_sides = float(self.get_value_for_user_setting('num_sides'))
        radius = self.get_value_for_user_setting('radius')
        direction = -1 if self.get_value_for_user_setting('direction') == 'clockwise' else 1

        if self.center is None:
            self.center = api.vehicle.get_position()
            print("new center {}".format(self.center))

        if self.desired_position is None:
            desired_angle = self.index / num_sides * M_2PI
            direction = np.array([cos(desired_angle), sin(desired_angle), 0])
            self.desired_position = self.center + radius * direction
            print("new desired position {}".format(self.desired_position))
            self.last_change_utime = api.utime

        if self.publish_downsampler.ready(api.utime):
            self.update_ar_scene(api)

        # Compute the distance between our goal and the vehicle
        position_delta = self.desired_position - api.vehicle.get_position()

        # Compute the time (in seconds) since we started trying to reach this position
        time_elapsed = (api.utime - self.last_change_utime) / 1e6

        # Advance to the next vertex if we reach the current position or we time out.
        if np.linalg.norm(position_delta) < 0.5 or time_elapsed > 10.0:
            # Clear state here so it gets set again on the next call to update()
            self.desired_position = None
            self.index = (self.index + direction) % num_sides
            self.set_needs_layout()

            # Schedule a call to Skill.get_onscreen_controls()
            # This will allow the title to update immediately to match the new vertex.
            self.set_needs_layout()

        else:
            # move to the desired position
            api.movement.set_desired_pos_nav(self.desired_position)
            # turn in the direction of the desired position
            api.movement.set_heading(core.azimuth(position_delta))
            # look in the vertical direction of the desired position
            api.movement.set_gimbal_pitch(-core.elevation(position_delta))
