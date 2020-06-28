import sys
import time
import argparse
from tqdm import tqdm
import pybullet
import pybullet_data

from blue_controller import BlueRobotController

# Constants
BUCLE_STEP = 0.016
POSITION_STEP = 0.1
ROTATION_STEP = 0.1
GRAVITY = -9.81

# Controller
steps = 0
robot_controller = None
left_position = None
left_orientation = None
right_position = None
right_orientation = None

# Events
########
def is_key_pressed(keys, key):
    if ord(key) in keys and keys[ord(key)] & pybullet.KEY_WAS_TRIGGERED:
        print("keyboard event: " + key + " pushed.")
        return True
    return False

def check_keyboard():
    global left_position
    global left_orientation
    global right_position
    global right_orientation

    keys = pybullet.getKeyboardEvents()

    # Quit
    if is_key_pressed(keys, 'q'):
        return True

    # Left position
    if is_key_pressed(keys, 'w'):
        left_position[0] += POSITION_STEP

    if is_key_pressed(keys, 'e'):
        left_position[1] += POSITION_STEP

    if is_key_pressed(keys, 'r'):
        left_position[2] += POSITION_STEP

    # Left rotation
    if is_key_pressed(keys, 's'):
        left_orientation[0] += ROTATION_STEP

    if is_key_pressed(keys, 'd'):
        left_orientation[0] += ROTATION_STEP

    if is_key_pressed(keys, 'f'):
        left_orientation[0] += ROTATION_STEP

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
    global steps
    global left_position
    global left_orientation
    global right_position
    global right_orientation

    steps = 0
    while True:
        # Get info from robot positions
        left_position = robot_controller.get_left_position()
        left_orientation = robot_controller.get_left_orientation()

        right_position = robot_controller.get_right_position()
        right_orientation = robot_controller.get_right_orientation()

        #print(left_position)
        # Process user inputs
        check_mouse()
        check_gamepad()
        abort = check_keyboard()

        #print(left_position)

        if abort:
            break

        # Controller BUG, changing position no set the new position on the render
        # Set info position
        robot_controller.set_left_position(left_position)
        robot_controller.set_left_orientation(left_orientation)
        robot_controller.set_right_position(right_position)
        robot_controller.set_right_orientation(right_orientation)

        robot_controller.check_clamp_control()

        # Render debug info
        robot_controller.debug_right_position(right_position)
        robot_controller.debug_left_position(left_position)

        steps += 1
        if steps % 100 == 0:
            robot_controller.print_robot_info()

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

