import pybullet

from blue import BlueRobot

class BlueRobotController():
    class PositionControl():
        def __init__(self, initial_position, initial_orientation, prefix):
            position_idx = []
            x, y, z = initial_position[0], initial_position[1], initial_position[2]
            offset = 1
            position_idx.append(pybullet.addUserDebugParameter('%s x' % prefix, x - offset, x + offset, x))
            position_idx.append(pybullet.addUserDebugParameter('%s y' % prefix, y - offset, y + offset, y))
            position_idx.append(pybullet.addUserDebugParameter('%s z' % prefix, 0.2, 2, z))

            orientation_idx = []
            orientation_idx.append(pybullet.addUserDebugParameter('%s euler 1' % prefix, -3.14, 3.14,
                                                                initial_orientation[0]))
            orientation_idx.append(pybullet.addUserDebugParameter('%s euler 2' % prefix, -3.14, 3.14,
                                                                initial_orientation[1]))
            orientation_idx.append(pybullet.addUserDebugParameter('%s euler 3' % prefix, -3.14, 3.14,
                                                                initial_orientation[2]))

            self.position_idx = position_idx
            self.orientation_idx = orientation_idx

        def get_position(self):
            orientation = [pybullet.readUserDebugParameter(idx) for idx in self.orientation_idx]
            position = [pybullet.readUserDebugParameter(idx) for idx in self.position_idx]
            return position, orientation

    class ClampControl():
        def __init__(self, prefix):
            self.id = pybullet.addUserDebugParameter('%s clamp' % prefix, 0, -1, 0)
            self.value = 0

        def close_clamp(self):
            return pybullet.readUserDebugParameter(self.id) % 2

    def __init__(self, path):
        self._robot = None
        self._right_position = None
        self._right_orientation = None
        self._left_position = None
        self._left_orientation = None
        self._right_control = None
        self._left_control = None
        self._right_clamp_control = None
        self._left_clamp_control = None

        self._robot = BlueRobot(path)
        self._robot.startup()

        # We may change this to put inside robot TODO
        self._right_control = self.PositionControl(*self._robot.get_right_arm_position(), prefix='right')
        self._right_clamp_control = self.ClampControl(prefix='right')

        self._left_control = self.PositionControl(*self._robot.get_left_arm_position(), prefix='left')
        self._left_clamp_control = self.ClampControl(prefix='left')
        self._robot.debug_arm_idx()

    def read_robot_attributes(self):
        self._right_position, self._right_orientation = self._right_control.get_position()
        self._left_position, self._left_orientation = self._left_control.get_position()

    def write_robot_attributes(self):
        self._robot.move_right_arm(self._right_position, self._right_orientation)
        self._robot.move_left_arm(self._left_position, self._left_orientation)

        # Debug only
        self.debug_position(self._right_position, self._robot.get_right_arm_position()[0])
        self.debug_position(self._left_position, self._robot.get_left_arm_position()[0])

    def print_robot_info(self):
        print("right arm:")
        print(self._right_position)
        print(self._right_orientation)
        print("left arm:")
        print(self._left_position)
        print(self._left_orientation)

    def debug_position(self, goal, source):
        pybullet.addUserDebugLine(
            goal, source, lineColorRGB=[1, 0, 0], lifeTime=1, lineWidth=2)

    def check_clamp_control(self):
        if self._right_clamp_control.close_clamp():
            self._robot.close_right_clamp()
        else:
            self._robot.open_right_clamp()
        if self._left_clamp_control.close_clamp():
            self._robot.close_left_clamp()
        else:
            self._robot.open_left_clamp()