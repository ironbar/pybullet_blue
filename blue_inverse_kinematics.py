import sys
import time
import argparse
from tqdm import tqdm
import pybullet
import pybullet_data

from blue_controller import BlueRobotController

# Constants
BUCLE_STEP = 0.016
GRAVITY = -9.81

# Controller
robot_controller = None

# Events
########
def is_key_pressed(keys, key):
    if ord(key) in keys and keys[ord(key)] & pybullet.KEY_WAS_TRIGGERED:
        print("keyboard event: " + key + " pushed.")
        return True
    return False

def check_keyboard():
    keys = pybullet.getKeyboardEvents()

    if is_key_pressed(keys, 'q'):
        return True

    if is_key_pressed(keys, 'j'):
        print("do stuff")

    return False

def check_mouse():
    mouseEvents = pybullet.getMouseEvents()
    for e in mouseEvents:
        eventType = e[0]
        mousePosX = e[1]
        mousePosY = e[2]
        buttonIndex = e[3]
        buttonState = e[4]

        button_type = "Unknown"
        button_id = "Unknown"
        button_action = "Unknown"

        if eventType == 1:
            button_type = "Movement"
        if eventType == 2:
            button_type = "Click"

        if buttonIndex == 0:
            button_id = "left"
        if buttonIndex == 1:
            button_id = "middle"
        if buttonIndex == 2:
            button_id = "right"

        if buttonState == 3:
            button_action = "down"
        if buttonState == 4:
            button_action = "release"

        if (eventType == 2):    # Only log click    
            print("Mouse " + button_id + " " + button_type + " " + button_action + " at: (" + str(mousePosX) + "," \
                + str(mousePosY) + ")" + " ")
            return True
    return False

def check_gamepad():
    return True

# Logic
#######
def run():
    """
    Run forever the robot (until q is pressed)
    """
    while True:
        robot_controller.read_robot_attributes()
        robot_controller.write_robot_attributes()

        robot_controller.check_clamp_control()
        check_mouse()
        check_gamepad()
        abort = check_keyboard()

        if abort:
            break

        time.sleep(BUCLE_STEP)
    print("Q key was pressed. Quit.")

def initialize_pybullet():
    pybullet.connect(pybullet.GUI)
    pybullet.setAdditionalSearchPath(pybullet_data.getDataPath())
    pybullet.setGravity(0, 0, GRAVITY)
    pybullet.setRealTimeSimulation(True)

    plane = pybullet.loadURDF("plane.urdf")

# Main
######
if __name__ == '__main__':
    parser = argparse.ArgumentParser(
        description='Allows to move each joint of the robot independently.',
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument('-r', '--robot_path', help='Path to the urdf model of the robot',
                        default='robots/blue_full_v2.urdf')
    args = parser.parse_args(sys.argv[1:])

    initialize_pybullet()
    robot_controller = BlueRobotController(args.robot_path)
    run()

