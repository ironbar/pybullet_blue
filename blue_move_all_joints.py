import sys
import argparse
import pybullet
import pybullet_data

from blue import BlueRobot

def main():
    args = parse_args()
    pybullet.connect(pybullet.GUI)
    pybullet.setAdditionalSearchPath(pybullet_data.getDataPath())
    plane = pybullet.loadURDF("plane.urdf")
    robot = BlueRobot(args.robot_path)
    robot.startup()
    pybullet.setGravity(0, 0, -9.81)
    pybullet.setRealTimeSimulation(1) #this makes the simulation real time

    param_idx = []
    for idx in range(robot.n_moving_joints):
        lower_limit, upper_limit = robot.lower_limits[idx], robot.upper_limits[idx]
        print('Joint %i has limits: (%.1f , %.1f)' % (idx, lower_limit, upper_limit))
        start_position = robot.rest_poses[idx]
        param_idx.append(pybullet.addUserDebugParameter('joint %i' % idx, lower_limit, upper_limit, start_position))

    while 1:
        target_positions = [pybullet.readUserDebugParameter(idx) for idx in param_idx]
        pybullet.setJointMotorControlArray(robot.id, robot.moving_joints_idx, pybullet.POSITION_CONTROL,
                                           targetPositions=target_positions)

def parse_args():
    parser = argparse.ArgumentParser(
        description='Allows to move each joint of the robot independently.',
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument('-r', '--robot_path', help='Path to the urdf model of the robot',
                        default='robots/blue_full_v2.urdf')
    return parser.parse_args(sys.argv[1:])

if __name__ == '__main__':
    main()
