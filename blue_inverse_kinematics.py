import os
import sys
import argparse
from tqdm import tqdm
import pybullet
import pybullet_data

from blue import BlueRobot
from utils import (
    debug_position, set_minimal_environment,
    add_table_with_objects_to_simulation)
from controllers import BlueSliderController, BlueVRController

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


def parse_args():
    description = """
    Control blue robot with inverse kinematics and VR

    If Using VR:
    Previously to launch the python script the App_PhysicsServer_SharedMemory_VR should be running.
    That app is inside bullet and it needs to be compiled. Then it is launched using steam. In
    my case is the line below.

    /home/guillermo/.steam/ubuntu12_32/steam-runtime/run.sh /media/guillermo/Data/MEGA/mimo/bullet3/build_cmake/examples/SharedMemory/App_PhysicsServer_SharedMemory_VR
    """
    parser = argparse.ArgumentParser(
        description=description,
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
