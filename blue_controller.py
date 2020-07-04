import pybullet

from blue import BlueRobot
from utils import PoseControl, ClampControl

class BlueRobotController():
    def __init__(self, path):
        self._robot = None
        self._right_control = None
        self._left_control = None
        self._right_clamp_control = None
        self._left_clamp_control = None

        self._robot = BlueRobot(path)
        self._robot.startup()

        # We may change this to put inside robot TODO
        self._right_control = PoseControl(*self._robot.get_right_arm_pose(), prefix='right')
        self._right_clamp_control = ClampControl(prefix='right')

        self._left_control = PoseControl(*self._robot.get_left_arm_position(), prefix='left')
        self._left_clamp_control = ClampControl(prefix='left')
        self._robot.debug_arm_idx()

    def get_left_position(self):
        left_position, _ = self._left_control.get_pose()
        return left_position

    def get_left_orientation(self):
        _, left_orientation = self._left_control.get_pose()
        return left_orientation

    def get_right_position(self):
        right_position, _ = self._right_control.get_pose()
        return right_position

    def get_right_orientation(self):
        _, right_orientation = self._right_control.get_pose()
        return right_orientation

    def set_left_position(self, position):
        _, orientation = self._left_control.get_pose()
        self._robot.move_left_arm(position, orientation)

    def set_left_orientation(self, orientation):
        position, _ = self._left_control.get_pose()
        self._robot.move_left_arm(position, orientation)

    def set_right_position(self, position):
        _, orientation = self._right_control.get_pose()
        self._robot.move_right_arm(position, orientation)

    def set_right_orientation(self, orientation):
        position, _ = self._right_control.get_pose()
        self._robot.move_right_arm(position, orientation)

    def print_robot_info(self):
        print("right arm:")
        print(self.get_right_position())
        print(self.get_right_orientation())
        print("left arm:")
        print(self.get_left_position())
        print(self.get_left_orientation())

    def check_clamp_control(self):
        if self._right_clamp_control.close_clamp():
            self._robot.close_right_clamp()
        else:
            self._robot.open_right_clamp()
        if self._left_clamp_control.close_clamp():
            self._robot.close_left_clamp()
        else:
            self._robot.open_left_clamp()

    def debug_left_position(self, goal):
        pybullet.addUserDebugLine(
            goal, self._robot.get_left_arm_position()[0], lineColorRGB=[1, 0, 0], lifeTime=1, lineWidth=2)

    def debug_right_position(self, goal):
        pybullet.addUserDebugLine(
            goal, self._robot.get_right_arm_pose()[0], lineColorRGB=[1, 0, 0], lifeTime=1, lineWidth=2)
