import os
import sys
import argparse
from tqdm import tqdm
import pybullet
import pybullet_data

from blue import BlueRobot
from utils import (
    PoseControl, ClampControl, debug_position, set_minimal_environment,
    add_table_with_objects_to_simulation)

DEFAULT_ROBOT_PATH = os.path.join(
    os.path.dirname(os.path.realpath(__file__)),
    'robots/blue_full_v2.urdf')

def main():
    args = parse_args()
    pybullet.connect(pybullet.GUI)
    set_minimal_environment()
    if args.add_toys:
        add_table_with_objects_to_simulation()

    robot = BlueRobot(args.robot_path)
    robot.startup()
    robot.debug_arm_idx()
    controller = BlueSliderController(robot)

    while 1:
        for _ in tqdm(range(1000), desc='Running simulation'):
            left_pose, right_pose, left_clamp, right_clamp = controller.get_poses_and_clamps()

            robot.move_right_arm(*right_pose)
            if args.debug_position: debug_position(right_pose[0], robot.get_right_arm_pose()[0])

            robot.move_left_arm(*left_pose)
            if args.debug_position: debug_position(left_pose[0], robot.get_left_arm_pose()[0])

            if right_clamp:
                robot.close_right_clamp()
            else:
                robot.open_right_clamp()
            if left_clamp:
                robot.close_left_clamp()
            else:
                robot.open_left_clamp()
            pybullet.stepSimulation()

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

def parse_args():
    parser = argparse.ArgumentParser(
        description='Control blue robot with inverse kinematics',
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument('-r', '--robot_path', help='Path to the urdf model of the robot',
                        default=DEFAULT_ROBOT_PATH)
    parser.add_argument('-d', '--debug_position', help='Draw lines between current position and goal position',
                        action='store_true')
    parser.add_argument('-t', '--add_toys', help='Add a table with toys for playing',
                        action='store_true')
    return parser.parse_args(sys.argv[1:])

if __name__ == '__main__':
    main()
