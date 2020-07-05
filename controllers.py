"""
Controllers for Blue Robot
"""
import pybullet

from utils import PoseControl, ClampControl

class BlueSliderController():
    """
    Slider Controller for blue robot
    """
    def __init__(self, robot):
        self.right_control = PoseControl(*robot.get_right_arm_pose(), prefix='right')
        self.rigth_clamp_control = ClampControl(prefix='right')
        self.left_control = PoseControl(*robot.get_left_arm_pose(), prefix='left')
        self.left_clamp_control = ClampControl(prefix='left')

    def get_poses_and_clamps(self):
        return self.left_control.get_pose(), self.right_control.get_pose(), \
               self.left_clamp_control.close_clamp(), self.rigth_clamp_control.close_clamp()


class BlueVRController():
    """
    VR Controller for blue robot

    Press the trackpad to start tracking the VR controllers and control the robot
    Press button at the top to stop tracking
    """
    def __init__(self, robot):
        self.left_pose = robot.get_left_arm_pose()
        self.right_pose = robot.get_right_arm_pose()
        self.left_clamp, self.right_clamp = 0, 0
        self.start_controlling = False

    def get_poses_and_clamps(self):
        events = pybullet.getVREvents()
        for event in events:
            controller_id, position, orientation = event[:3]
            buttons = event[6]
            trigger_button = buttons[33]
            stop_tracking_button = buttons[1]
            trackpad_button = buttons[32]

            if trackpad_button and not self.start_controlling:
                print('Now the robot is being controlled')
                self.start_controlling = True

            if not self.start_controlling:
                continue

            if controller_id % 2 == 0 and not stop_tracking_button:
                self.left_pose = position, orientation
                self.left_clamp = trigger_button
            elif controller_id % 2 == 1 and not stop_tracking_button:
                self.right_pose = position, orientation
                self.right_clamp = trigger_button

        return self.left_pose, self.right_pose, self.left_clamp, self.right_clamp
