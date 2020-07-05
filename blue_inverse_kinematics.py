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
    if args.use_vr:
        pybullet.connect(pybullet.SHARED_MEMORY)
        pybullet.resetSimulation()
    else:
        pybullet.connect(pybullet.GUI)
    set_minimal_environment()
    if args.add_toys:
        add_table_with_objects_to_simulation()

    robot = BlueRobot(args.robot_path)
    robot.startup()
    robot.debug_arm_idx()
    if args.use_vr:
        controller = BlueVRController(robot)
    else:
        controller = BlueSliderController(robot)

    while 1:
        for _ in tqdm(range(1000), desc='Running simulation'):
            left_pose, right_pose, left_clamp, right_clamp = controller.get_poses_and_clamps()
            robot.control(left_pose, right_pose, left_clamp, right_clamp, args.debug_position)
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
    parser.add_argument('-v', '--use_vr', help='Use VR controller',
                        action='store_true')
    return parser.parse_args(sys.argv[1:])

if __name__ == '__main__':
    main()
