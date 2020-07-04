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
DEFAULT_ROBOT_PATH = os.path.join(SCRIPT_DIR, '../robots/blue_left_v2.urdf')

sys.path.append(os.path.join(SCRIPT_DIR, '..'))
from blue import BlueArm
from utils import PoseControl, ClampControl, debug_position, set_minimal_environment



def main():
    args = parse_args()
    if args.use_gui:
        pybullet.connect(pybullet.GUI)
    else:
        pybullet.connect(pybullet.DIRECT)
    pybullet.setAdditionalSearchPath(pybullet_data.getDataPath())
    pybullet.loadURDF("plane.urdf")
    pybullet.setGravity(0, 0, -9.81)

    robot = BlueArm(args.robot_path)
    robot.startup(is_real_time=False)

    speed = measure_speed(robot, distance_threshold=1e-1)
    print('The speed when the threshold is 1e-1 is: %i' % speed)
    speed = measure_speed(robot, distance_threshold=1e-2)
    print('The speed when the threshold is 1e-2 is: %i' % speed)

def measure_speed(robot, distance_threshold, n_cycles=5, startup_cycles=20):
    _, orientation = robot.get_pose()
    source, goal = [0.3, 0.3, 0.5], [0.3, -0.3, 0.5]
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
                        default=DEFAULT_ROBOT_PATH)
    parser.add_argument('-v', '--use_gui', help='Use GUI',
                        action='store_true')
    return parser.parse_args(sys.argv[1:])

if __name__ == '__main__':
    main()
