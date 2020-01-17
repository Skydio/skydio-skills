# Prepare for Python3 conversion
from __future__ import absolute_import
from __future__ import print_function
import json
import numpy as np

# The base class for all Skydio2 Skills
from vehicle.skills.base_skill import Skill


class MotionCommand(object):

    def __init__(self, utime, data):
        self.utime = utime

        velx, vely, velz, yaw_rate, pitch_rate = data
        vel_body = np.array([velx, vely, velz])
        self.vel_body = vel_body
        self.yaw_rate = yaw_rate
        self.pitch_rate = pitch_rate


COMMAND_TIMEOUT = 1.0  # [s] Number of seconds to keep executing a command.
# This prevents the vehicle from continuing to fly after WiFi loss


class RemoteControl(Skill):
    """ Control the vehicle from an separate computer via WiFi or USB ethernet. """

    def __init__(self):
        super(RemoteControl, self).__init__()
        self.command = None

    def update(self, api):
        # Don't allow subject tracking in this mode.
        api.subject.cancel_subject_tracking(api.utime)

        # Don't allow phone movements.
        api.phone.disable_movement_commands()

        # Set the upper-limit for vehicle speed
        api.movement.set_max_speed(12.0)

        # Publish a status message to the phone with some data in it.
        status = {}
        status['speed'] = api.vehicle.get_speed()
        status['position'] = list(api.vehicle.get_position())
        api.custom_comms.publish_status(json.dumps(status))

        if not self.command:
            # Nothing to do
            return

        elapsed_seconds = (api.utime - self.command.utime) / 1e6
        if elapsed_seconds > COMMAND_TIMEOUT:
            # The command has expired. Stop the vehicle.
            api.movement.set_desired_vel_body(np.array([0, 0, 0]))
            api.movement.set_heading_rate(0)
        else:
            # Keep applying the current command.
            api.movement.set_desired_vel_body(self.command.vel_body)
            api.movement.set_heading_rate(self.command.yaw_rate)

            # Adjust the pitch
            # NOTE: there is currently not an API for pitch rate, so we approximate here.
            pitch = api.vehicle.get_gimbal_pitch()
            api.movement.set_gimbal_pitch(pitch + self.command.pitch_rate)

    def handle_rpc(self, api, message):
        """ Process an incoming request and extract the motion command. """
        # Assume json encoding.
        data = json.loads(message)
        if 'move' in data:
            self.command = MotionCommand(api.utime, data['move'])
