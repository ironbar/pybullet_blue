import sys
import time
import argparse
from tqdm import tqdm
import pybullet
import pybullet_data

from blue import BlueRobot

# Keyboard
qKey = ord('q')

def main():
    args = parse_args()
    pybullet.connect(pybullet.GUI)
    pybullet.setAdditionalSearchPath(pybullet_data.getDataPath())
    plane = pybullet.loadURDF("plane.urdf")
    pybullet.setGravity(0, 0, -9.81)
    pybullet.setRealTimeSimulation(1) #this makes the simulation real time

    robot = BlueRobot(args.robot_path)

    robot.go_to_rest_pose()
    for _ in tqdm(range(10), desc='startup'):
        time.sleep(0.01)

    right_control = PositionControl(*robot.get_right_arm_position(), prefix='right')
    rigth_clamp_control = ClampControl(prefix='right')

    left_control = PositionControl(*robot.get_left_arm_position(), prefix='left')
    left_clamp_control = ClampControl(prefix='left')
    robot.debug_arm_idx()

    

    while 1:
        position, orientation = right_control.get_position()
        robot.move_right_arm(position, orientation)
        debug_position(position, robot.get_right_arm_position()[0])

        position, orientation = left_control.get_position()
        robot.move_left_arm(position, orientation)
        debug_position(position, robot.get_left_arm_position()[0])

        if rigth_clamp_control.close_clamp():
            robot.close_right_clamp()
        else:
            robot.open_right_clamp()
        if left_clamp_control.close_clamp():
            robot.close_left_clamp()
        else:
            robot.open_left_clamp()

        check_mouse()
        check_gamepad()
        abort = check_keyboard()
        if abort:
            break

        time.sleep(0.01)

def check_keyboard():
    keys = pybullet.getKeyboardEvents()
    if qKey in keys and keys[qKey]&pybullet.KEY_WAS_TRIGGERED:
        print("keyboard event: q pushed")
        return True
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

def parse_args():
    parser = argparse.ArgumentParser(
        description='Allows to move each joint of the robot independently.',
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument('-r', '--robot_path', help='Path to the urdf model of the robot',
                        default='robots/blue_full_v2.urdf')
    return parser.parse_args(sys.argv[1:])

if __name__ == '__main__':
    main()
