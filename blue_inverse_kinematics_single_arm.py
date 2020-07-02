"""
Control a single blue robot arm with inverse kinematics
"""
import os
import sys
import argparse
import pybullet
import pybullet_data

from blue import BlueArm
from utils import PoseControl, ClampControl, debug_position, set_minimal_environment

DEFAULT_ROBOT_PATH = os.path.join(
    os.path.dirname(os.path.realpath(__file__)),
    'robots/blue_left_v2.urdf')

def main():
    args = parse_args()
    pybullet.connect(pybullet.GUI)
    set_minimal_environment()

    robot = BlueArm(args.robot_path)
    robot.startup()

    pose_control = PoseControl(*robot.get_pose(), prefix='right')
    clamp_control = ClampControl(prefix='right')
    robot.debug_arm_idx()

    while 1:
        position, orientation = pose_control.get_pose()
        robot.move(position, orientation)
        debug_position(position, robot.get_pose()[0])

        if clamp_control.close_clamp():
            robot.close_clamp()
        else:
            robot.open_clamp()

def parse_args():
    parser = argparse.ArgumentParser(
        description='Control a single blue robot arm with inverse kinematics',
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument('-r', '--robot_path', help='Path to the urdf model of the robot',
                        default=DEFAULT_ROBOT_PATH)
    return parser.parse_args(sys.argv[1:])

if __name__ == '__main__':
    main()
