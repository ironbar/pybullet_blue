import sys
import time
import argparse
from tqdm import tqdm
import pybullet
import pybullet_data

from blue import BlueRobot

# Constants
BUCLE_STEP = 0.016

# Robot
robot = None
right_position = None
right_orientation = None
left_position = None
left_orientation = None
right_control = None
left_control = None
right_clamp_control = None
left_clamp_control = None

# Keyboard
qKey = ord('q')
jKey = ord('j')

def check_keyboard():
    keys = pybullet.getKeyboardEvents()
    if qKey in keys and keys[qKey]&pybullet.KEY_WAS_TRIGGERED:
        print("keyboard event: q pushed")
        return True
    if jKey in keys and keys[jKey]&pybullet.KEY_WAS_TRIGGERED:
        print("keyboard event: j pushed")
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

def read_robot_attributes():
    global right_position, right_orientation
    global left_position, left_orientation
    right_position, right_orientation = right_control.get_position()
    left_position, left_orientation = left_control.get_position()

def write_robot_attributes():
    robot.move_right_arm(right_position, right_orientation)
    robot.move_left_arm(left_position, left_orientation)

    # Debug only
    debug_position(right_position, robot.get_right_arm_position()[0])
    debug_position(left_position, robot.get_left_arm_position()[0])

def print_robot_info():
    print("right arm:")
    print(right_position)
    print(right_orientation)
    print("left arm:")
    print(left_position)
    print(left_orientation)

def debug_position(goal, source):
    pybullet.addUserDebugLine(
        goal, source, lineColorRGB=[1, 0, 0], lifeTime=1, lineWidth=2)

class PositionControl():

    def __init__(self, initial_position, initial_orientation, prefix):
        position_idx = []
        x, y, z = initial_position[0], initial_position[1], initial_position[2]
        offset = 1
        position_idx.append(pybullet.addUserDebugParameter('%s x' % prefix, x - offset, x + offset, x))
        position_idx.append(pybullet.addUserDebugParameter('%s y' % prefix, y - offset, y + offset, y))
        position_idx.append(pybullet.addUserDebugParameter('%s z' % prefix, 0.2, 2, z))

        orientation_idx = []
        orientation_idx.append(pybullet.addUserDebugParameter('%s euler 1' % prefix, -3.14, 3.14,
                                                              initial_orientation[0]))
        orientation_idx.append(pybullet.addUserDebugParameter('%s euler 2' % prefix, -3.14, 3.14,
                                                              initial_orientation[1]))
        orientation_idx.append(pybullet.addUserDebugParameter('%s euler 3' % prefix, -3.14, 3.14,
                                                              initial_orientation[2]))

        self.position_idx = position_idx
        self.orientation_idx = orientation_idx

    def get_position(self):
        orientation = [pybullet.readUserDebugParameter(idx) for idx in self.orientation_idx]
        position = [pybullet.readUserDebugParameter(idx) for idx in self.position_idx]
        return position, orientation


class ClampControl():

    def __init__(self, prefix):
        self.id = pybullet.addUserDebugParameter('%s clamp' % prefix, 0, -1, 0)
        self.value = 0

    def close_clamp(self):
        return pybullet.readUserDebugParameter(self.id) % 2

def check_clamp_control():
    if right_clamp_control.close_clamp():
        robot.close_right_clamp()
    else:
        robot.open_right_clamp()
    if left_clamp_control.close_clamp():
        robot.close_left_clamp()
    else:
        robot.open_left_clamp()

def run():
    """
    Run forever the robot (until q is pressed)
    """
    while True:
        read_robot_attributes()
        write_robot_attributes()

        check_clamp_control()
        check_mouse()
        check_gamepad()
        abort = check_keyboard()
        if abort:
            break

        time.sleep(BUCLE_STEP)

def parse_args():
    parser = argparse.ArgumentParser(
        description='Allows to move each joint of the robot independently.',
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument('-r', '--robot_path', help='Path to the urdf model of the robot',
                        default='robots/blue_full_v2.urdf')
    return parser.parse_args(sys.argv[1:])

if __name__ == '__main__':
    args = parse_args()

    # Initialize Engine
    pybullet.connect(pybullet.GUI)
    pybullet.setAdditionalSearchPath(pybullet_data.getDataPath())
    plane = pybullet.loadURDF("plane.urdf")
    pybullet.setGravity(0, 0, -9.81)
    pybullet.setRealTimeSimulation(1) #this makes the simulation real time

    # Initialize robot
    robot = BlueRobot(args.robot_path)
    robot.startup()

    right_control = PositionControl(*robot.get_right_arm_position(), prefix='right')
    right_clamp_control = ClampControl(prefix='right')

    left_control = PositionControl(*robot.get_left_arm_position(), prefix='left')
    left_clamp_control = ClampControl(prefix='left')
    robot.debug_arm_idx()

    run()

