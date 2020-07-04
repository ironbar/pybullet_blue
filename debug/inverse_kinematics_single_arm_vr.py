"""
Control a single blue robot arm with inverse kinematics using VR
"""
import os
import sys
import argparse
import pybullet
import pybullet_data
from tqdm import tqdm

SCRIPT_DIR = os.path.dirname(os.path.realpath(__file__))
DEFAULT_BLUE_ROBOT_PATH = os.path.join(SCRIPT_DIR, '../robots/blue_left_v2.urdf')
DEFAULT_KUKA_ROBOT_PATH = os.path.join(SCRIPT_DIR, '../robots/lbr_iiwa_14_r820.urdf')

sys.path.append(os.path.join(SCRIPT_DIR, '..'))
from blue import BlueArm
from utils import PoseControl, ClampControl, debug_position, set_minimal_environment
from kuka import Kuka



def main():
    args = parse_args()
    pybullet.connect(pybullet.SHARED_MEMORY)
    pybullet.resetSimulation()
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

    robot.debug_arm_idx()
    robot.get_mass()

    while 1:
        for _ in tqdm(range(1000), desc='Running simulation'):
            events = pybullet.getVREvents()
            for event in events:
                _, position, orientation = event[:3]
                buttons = event[6]
                trigger_button = buttons[33]

            robot.move(position, orientation)
            if args.debug_position:
                debug_position(position, robot.get_pose()[0])
            if not args.use_kuka:
                if trigger_button:
                    robot.close_clamp()
                else:
                    robot.open_clamp()
            pybullet.stepSimulation()

def parse_args():
    description = """
    Control a single blue robot arm with inverse kinematics using VR
    Previously to launch the python script the App_PhysicsServer_SharedMemory_VR should be running.
    That app is inside bullet and it needs to be compiled. Then it is launched using steam. In
    my case is the line below.

    /home/guillermo/.steam/ubuntu12_32/steam-runtime/run.sh /media/guillermo/Data/MEGA/mimo/bullet3/build_cmake/examples/SharedMemory/App_PhysicsServer_SharedMemory_VR
    """
    parser = argparse.ArgumentParser(
        description=description,
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument('-r', '--robot_path', help='Path to the urdf model of the robot',
                        default=None)
    parser.add_argument('-k', '--use_kuka', help='Use Kuka instead of Blue',
                        action='store_true')
    parser.add_argument('-d', '--debug_position', help='Draw lines between current position and goal position',
                        action='store_true')
    return parser.parse_args(sys.argv[1:])

if __name__ == '__main__':
    main()
