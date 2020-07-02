import os
import sys
import argparse
import pybullet
import pybullet_data

from blue import BlueRobot
from utils import PoseControl, ClampControl, debug_position, set_minimal_environment

DEFAULT_ROBOT_PATH = os.path.join(
    os.path.dirname(os.path.realpath(__file__)),
    'robots/blue_full_v2.urdf')

def main():
    args = parse_args()
    pybullet.connect(pybullet.GUI)
    set_minimal_environment()

    robot = BlueRobot(args.robot_path)
    robot.startup()

    right_control = PoseControl(*robot.get_right_arm_position(), prefix='right')
    rigth_clamp_control = ClampControl(prefix='right')
    left_control = PoseControl(*robot.get_left_arm_position(), prefix='left')
    left_clamp_control = ClampControl(prefix='left')
    robot.debug_arm_idx()

    while 1:
        position, orientation = right_control.get_pose()
        robot.move_right_arm(position, orientation)
        debug_position(position, robot.get_right_arm_position()[0])

        position, orientation = left_control.get_pose()
        robot.move_left_arm(position, orientation)
        debug_position(position, robot.get_left_arm_position()[0])

        if rigth_clamp_control.close_clamp():
            robot.close_right_clamp()
        else:
            robot.open_right_clamp()
        if left_clamp_control.close_clamp():
            robot.close_left_clamp()
        else:
            robot.open_left_clamp()

def parse_args():
    parser = argparse.ArgumentParser(
        description='Control blue robot with inverse kinematics',
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument('-r', '--robot_path', help='Path to the urdf model of the robot',
                        default=DEFAULT_ROBOT_PATH)
    return parser.parse_args(sys.argv[1:])

if __name__ == '__main__':
    main()
