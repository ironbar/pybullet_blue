"""
Control a single blue robot arm with inverse kinematics
"""
import os
import sys
import argparse
import pybullet
import pybullet_data

SCRIPT_DIR = os.path.dirname(os.path.realpath(__file__))
DEFAULT_BLUE_ROBOT_PATH = os.path.join(SCRIPT_DIR, '../robots/blue_left_v2.urdf')
DEFAULT_KUKA_ROBOT_PATH = os.path.join(SCRIPT_DIR, '../robots/lbr_iiwa_14_r820.urdf')

sys.path.append(os.path.join(SCRIPT_DIR, '..'))
from blue import BlueArm
from utils import PoseControl, ClampControl, debug_position, set_minimal_environment
from kuka import Kuka



def main():
    args = parse_args()
    pybullet.connect(pybullet.GUI)
    pybullet.setAdditionalSearchPath(pybullet_data.getDataPath())
    pybullet.loadURDF("plane.urdf")
    pybullet.setGravity(0, 0, -9.81)

    if args.use_kuka:
        robot_path = args.robot_path or DEFAULT_KUKA_ROBOT_PATH
        robot = Kuka(robot_path)
    else:
        robot_path = args.robot_path or DEFAULT_BLUE_ROBOT_PATH
        robot = BlueArm(robot_path)
    robot.startup(is_real_time=False)

    pose_control = PoseControl(*robot.get_pose(), prefix='right')
    if not args.use_kuka:
        clamp_control = ClampControl(prefix='right')
    robot.debug_arm_idx()
    robot.get_mass()

    while 1:
        position, orientation = pose_control.get_pose()
        robot.move(position, orientation)
        debug_position(position, robot.get_pose()[0])

        if not args.use_kuka:
            if clamp_control.close_clamp():
                robot.close_clamp()
            else:
                robot.open_clamp()
        pybullet.stepSimulation()

def parse_args():
    parser = argparse.ArgumentParser(
        description='Control a single blue robot arm with inverse kinematics',
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument('-r', '--robot_path', help='Path to the urdf model of the robot',
                        default=None)
    parser.add_argument('-k', '--use_kuka', help='Use Kuka instead of Blue',
                        action='store_true')
    return parser.parse_args(sys.argv[1:])

if __name__ == '__main__':
    main()
