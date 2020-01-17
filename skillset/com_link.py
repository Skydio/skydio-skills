# Prepare for Python3 conversion
from __future__ import absolute_import
from __future__ import print_function
import json
import numpy as np

# Import from the Skydio2 skills sdk
from vehicle.skills.base_skill import Skill
from vehicle.skills.util.ui import UiButton
from vehicle.skills.util.motions.goto_motion import GotoMotion
from vehicle.skills.util.transform import Transform


class ComLink(Skill):
    """ Communicate with a client device over HTTP.

    This skill displays basic information on the phone, but also
    sends and receives data from an external python script.

    See skydio_client.py
    """

    def __init__(self):
        super(ComLink, self).__init__()
        self.data = {}
        self.pressed = False
        self.motion = None

    def handle_rpc(self, api, message):
        """ A client send custom message to this Skill. """

        # Serialization format is arbitrary. Here we use assume the client is sending json
        self.data = json.loads(message)

        # Populate a dictionary with response values
        response = {}

        # Vehicle states
        position = api.vehicle.get_position()
        if position is not None:
            response['position'] = position.tolist()
        speed = api.vehicle.get_speed()
        if speed is not None:
            response['speed'] = speed

        # Button status
        if self.pressed:
            response['pressed_button'] = True
            # Clear the value now that we've sent it to the client
            self.pressed = False

        # Move forward X meters
        if 'forward' in self.data:
            # Move to a position in front of the vehicle's main camera.
            nav_T_cam = api.vehicle.get_camera_trans()
            cam_t_point = np.array([self.data['forward'], 0, 0])
            nav_t_point = nav_T_cam * cam_t_point
            nav_T_goal = Transform(nav_T_cam.rotation(), nav_t_point)

            # Get optional speed parameter.
            speed = self.data.get('speed', 2.0)

            # Create Motion object that will manage vehicle position updates over time.
            self.motion = GotoMotion(nav_T_goal, params=dict(speed=speed))

        # Update the layout every time we get a request.
        self.set_needs_layout()

        # Serialization format is arbitrary. Here we send json.
        return json.dumps(response)

    def button_pressed(self, api, button_id, source):  # pylint: disable=unused-argument
        """ The user pressed a button in the Skydio app. """
        # Pressing the STOP button should cancel the motion.
        if button_id == 'stop':
            self.motion = None
        elif button_id == 'send':
            # Record for later
            self.pressed = True
        else:
            print('unknown button {}'.format(button_id))

    def get_onscreen_controls(self, api):
        """ Configure the ui. """
        controls = {}
        if self.motion:
            controls['show_stop'] = True
            controls['height_slider_enabled'] = False
        else:
            controls['show_stop'] = False
            controls['height_slider_enabled'] = True

        title = type(self.motion) if self.motion else ''
        controls['title'] = str(self.data.get('title', title))
        controls['detail'] = str(self.data.get('detail', ''))
        if not self.pressed:
            controls['buttons'] = [UiButton('send', label='Ping')]
        else:
            controls['buttons'] = []
        return controls

    def update(self, api):
        """ Control the vehicle. """
        # If we're executing a motion, disable phone commands and update the motion
        if self.motion:
            api.phone.disable_movement_commands()
            self.motion.update(api)

            api.planner.settings.obstacle_safety = 1.0
            api.movement.set_max_speed(10.0)

            # When the motion completes, clear it and update the UI.
            if self.motion.done:
                print("motion complete")
                self.motion = None
                self.set_needs_layout()
        else:
            # Otherwise, just listen to the phone.
            api.phone.enable_movement_commands()
