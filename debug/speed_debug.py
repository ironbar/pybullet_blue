"""
Control a single blue robot arm with inverse kinematics
"""
import os
import sys
import argparse
import pybullet
import pybullet_data
import numpy as np

SCRIPT_DIR = os.path.dirname(os.path.realpath(__file__))
DEFAULT_BLUE_ROBOT_PATH = os.path.join(SCRIPT_DIR, '../robots/blue_left_v2.urdf')
DEFAULT_KUKA_ROBOT_PATH = os.path.join(SCRIPT_DIR, '../robots/lbr_iiwa_14_r820.urdf')

sys.path.append(os.path.join(SCRIPT_DIR, '..'))
from blue import BlueArm
from utils import PoseControl, ClampControl, debug_position, set_minimal_environment
from kuka import Kuka


def main():
    args = parse_args()
    if args.use_gui:
        pybullet.connect(pybullet.GUI)
    else:
        pybullet.connect(pybullet.DIRECT)
    set_minimal_environment()

    if args.use_kuka:
        robot_path = args.robot_path or DEFAULT_KUKA_ROBOT_PATH
        robot = Kuka(robot_path)
    else:
        robot_path = args.robot_path or DEFAULT_BLUE_ROBOT_PATH
        robot = BlueArm(robot_path)
    robot.startup(is_real_time=False)

    speed = measure_speed(robot, distance_threshold=1e-1)
    print('The speed when the threshold is 1e-1 is: %i' % speed)
    # Commented because kuka is not able to achieve that accuracy
    # speed = measure_speed(robot, distance_threshold=1e-2)
    # print('The speed when the threshold is 1e-2 is: %i' % speed)

def measure_speed(robot, distance_threshold, n_cycles=10, startup_cycles=20):
    _, orientation = robot.get_pose()
    source, goal = [0.3, 0.3, 0.6], [0.3, -0.3, 0.6]
    # do a first cycles for startup
    for _ in range(startup_cycles):
        go_to_pose(robot, source, orientation, distance_threshold=distance_threshold)
        go_to_pose(robot, goal, orientation, distance_threshold=distance_threshold)
    # now measure the steps
    steps = []
    for _ in range(n_cycles):
        steps.append(go_to_pose(robot, source, orientation, distance_threshold=distance_threshold))
        steps.append(go_to_pose(robot, goal, orientation, distance_threshold=distance_threshold))
    print(steps[::2], steps[1::2])
    speed = np.mean(steps)
    return speed

def go_to_pose(robot, position, orientation, max_steps=10000, distance_threshold=1e-2):
    for step_idx in range(max_steps):
        robot.move(position, orientation)
        current_position = robot.get_pose()[0]
        debug_position(position, current_position)
        pybullet.stepSimulation()
        distance = compute_distance(position, current_position)
        if distance < distance_threshold:
            break
    return step_idx

def compute_distance(position, goal):
    return np.sqrt(np.sum((np.array(position) - np.array(goal))**2))

def parse_args():
    parser = argparse.ArgumentParser(
        description='Control a single blue robot arm with inverse kinematics',
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument('-r', '--robot_path', help='Path to the urdf model of the robot',
                        default=None)
    parser.add_argument('-v', '--use_gui', help='Use GUI',
                        action='store_true')
    parser.add_argument('-k', '--use_kuka', help='Use Kuka instead of Blue',
                        action='store_true')
    return parser.parse_args(sys.argv[1:])

if __name__ == '__main__':
    main()
