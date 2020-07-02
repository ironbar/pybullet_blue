"""
Utilities to control Blue robot
"""
import time
from tqdm import tqdm
import pybullet


class Robot():
    def __init__(self):
        self.id = None
        self.kinematics_kwargs = {}
        self.moving_joints_idx = []
        self.rest_poses = None

    def _get_link_state(self, link_idx):
        return get_link_state(self.id, link_idx)

    def _inverse_kinematics(self, link_idx, position, orientation):
        if len(orientation) == 3:
            orientation = pybullet.getQuaternionFromEuler(orientation)
        target_positions = pybullet.calculateInverseKinematics(
            self.id,
            endEffectorLinkIndex=link_idx,
            targetPosition=position,
            targetOrientation=orientation,
            **self.kinematics_kwargs)
        return target_positions

    def get_motor_positions(self):
        return [pybullet.getJointState(self.id, idx)[0] for idx in self.moving_joints_idx]

    def go_to_rest_pose(self):
        pybullet.setJointMotorControlArray(
            self.id, self.moving_joints_idx, pybullet.POSITION_CONTROL,
            targetPositions=self.rest_poses)

    def startup(self):
        for _ in tqdm(range(10), desc='startup'):
            self.go_to_rest_pose()
            time.sleep(0.01)


class BlueRobot(Robot):
    RIGHT_ARM_LINK_IDX = 7
    LEFT_ARM_LINK_IDX = 24

    def __init__(self, robot_path):
        flags = pybullet.URDF_USE_SELF_COLLISION | pybullet.URDF_USE_SELF_COLLISION_EXCLUDE_ALL_PARENTS
        self.id = pybullet.loadURDF(robot_path, [0, 0, 1], useFixedBase=1, flags=flags)
        self.kinematics_kwargs, self.moving_joints_idx = getJointRanges(self.id, False)
        self.lower_limits = self.kinematics_kwargs['lowerLimits']
        self.upper_limits = self.kinematics_kwargs['upperLimits']
        self.rest_poses = self.kinematics_kwargs['restPoses']
        self.n_moving_joints = len(self.moving_joints_idx)

    def get_right_arm_position(self):
        return self._get_link_state(self.RIGHT_ARM_LINK_IDX)

    def get_left_arm_position(self):
        return self._get_link_state(self.LEFT_ARM_LINK_IDX)

    def move_right_arm(self, position, orientation):
        target_positions = self._inverse_kinematics(
            self.RIGHT_ARM_LINK_IDX, position, orientation)
        pybullet.setJointMotorControlArray(
            self.id, self.moving_joints_idx[:7], pybullet.POSITION_CONTROL,
            targetPositions=target_positions[:7],
            # targetVelocities=[0.1]*7,
            positionGains=[1]*7,
            # velocityGains=[1e-3]*7,
            )

    def move_left_arm(self, position, orientation):
        target_positions = self._inverse_kinematics(
            self.LEFT_ARM_LINK_IDX, position, orientation)
        pybullet.setJointMotorControlArray(
            self.id, self.moving_joints_idx[12:19], pybullet.POSITION_CONTROL,
            targetPositions=target_positions[12:19],
            positionGains=[1]*7
            )

    def close_right_clamp(self):
        pybullet.setJointMotorControlArray(
            self.id, self.moving_joints_idx[8:12], pybullet.POSITION_CONTROL,
            targetPositions=[1, 0, 1, 0])

    def open_right_clamp(self):
        pybullet.setJointMotorControlArray(
            self.id, self.moving_joints_idx[8:12], pybullet.POSITION_CONTROL,
            targetPositions=[0]*4)

    def close_left_clamp(self):
        pybullet.setJointMotorControlArray(
            self.id, self.moving_joints_idx[20:24], pybullet.POSITION_CONTROL,
            targetPositions=[1, 0, 1, 0])

    def open_left_clamp(self):
        pybullet.setJointMotorControlArray(
            self.id, self.moving_joints_idx[20:24], pybullet.POSITION_CONTROL,
            targetPositions=[0]*4)

    def debug_arm_idx(self):
        # press 'w' to see this links highlighted
        pybullet.setDebugObjectColor(self.id, self.RIGHT_ARM_LINK_IDX, [1, 0, 0])
        pybullet.setDebugObjectColor(self.id, self.LEFT_ARM_LINK_IDX, [1, 0, 0])

    def _print_motor_positions_for_debug(self, target_positions):
        sep = '  \t'
        print('idx               ', sep.join([str(x) for x, _ in enumerate(target_positions)]))
        print('target_positions  ', sep.join([str(round(x, 2)) for x in target_positions]))
        current_positions = self.get_motor_positions()
        print('current_positions ', sep.join([str(round(x, 2)) for x in current_positions]))
        print('diff              ', sep.join([str(round(x-y, 2)) for x, y in zip(current_positions, target_positions)]))
        print('lower_limit       ', sep.join([str(round(x, 2)) for x in self.lower_limits]))
        print('upper_limit       ', sep.join([str(round(x, 2)) for x in self.upper_limits]))
        print()


class BlueArm(Robot):
    LINK_IDX = 7

    def __init__(self, robot_path):
        flags = pybullet.URDF_USE_SELF_COLLISION | pybullet.URDF_USE_SELF_COLLISION_EXCLUDE_ALL_PARENTS
        self.id = pybullet.loadURDF(robot_path, [0, 0, 0], useFixedBase=1, flags=flags)
        self.kinematics_kwargs, self.moving_joints_idx = getJointRanges(self.id, False)
        self.lower_limits = self.kinematics_kwargs['lowerLimits']
        self.upper_limits = self.kinematics_kwargs['upperLimits']
        self.rest_poses = self.kinematics_kwargs['restPoses']
        self.n_moving_joints = len(self.moving_joints_idx)

    def get_pose(self):
        return self._get_link_state(self.LINK_IDX)

    def move(self, position, orientation):
        target_positions = self._inverse_kinematics(
            self.LINK_IDX, position, orientation)
        pybullet.setJointMotorControlArray(
            self.id, self.moving_joints_idx[:7], pybullet.POSITION_CONTROL,
            targetPositions=target_positions[:7],
            # targetVelocities=[0.1]*7,
            positionGains=[1]*7,
            # velocityGains=[1e-3]*7,
            )

    def close_clamp(self):
        pybullet.setJointMotorControlArray(
            self.id, self.moving_joints_idx[8:12], pybullet.POSITION_CONTROL,
            targetPositions=[1, 0, 1, 0])

    def open_clamp(self):
        pybullet.setJointMotorControlArray(
            self.id, self.moving_joints_idx[8:12], pybullet.POSITION_CONTROL,
            targetPositions=[0]*4)

    def debug_arm_idx(self):
        # press 'w' to see this links highlighted
        pybullet.setDebugObjectColor(self.id, self.LINK_IDX, [1, 0, 0])


def getJointRanges(bodyId, includeFixed=False):
    """
    https://github.com/erwincoumans/pybullet_robots/blob/master/baxter_ik_demo.py

    Parameters
    ----------
    bodyId : int
    includeFixed : bool
    Returns
    -------
    lowerLimits : [ float ] * numDofs
    upperLimits : [ float ] * numDofs
    jointRanges : [ float ] * numDofs
    restPoses : [ float ] * numDofs
    """

    lowerLimits, upperLimits, jointRanges, restPoses = [], [], [], []
    moving_joints_idx = []

    numJoints = pybullet.getNumJoints(bodyId)

    for i in range(numJoints):
        jointInfo = pybullet.getJointInfo(bodyId, i)

        if includeFixed or jointInfo[3] > -1:

            ll, ul = jointInfo[8:10]
            jr = ul - ll

            # For simplicity, assume resting state == initial state
            # rp = pybullet.getJointState(bodyId, i)[0]
            # Instead of that I will define a better rest position
            # This has clearly improved the Inverse kinematics calculation
            # However on v2 version of blue I have increased the range of the motors and probably
            # it is not necessary anymore. Or I may find a better rest position.
            if ul == 0:
                rp = ul - jr/4
            elif ul == -1:
                rp = 0
            else:
                rp = ll + jr/2

            lowerLimits.append(ll)
            upperLimits.append(ul)
            jointRanges.append(jr)
            restPoses.append(rp)
            moving_joints_idx.append(i)

    output = dict(
        lowerLimits=lowerLimits, upperLimits=upperLimits,
        jointRanges=jointRanges, restPoses=restPoses)
    return output, moving_joints_idx


def get_link_state(robot, link_idx):
    ret = pybullet.getLinkState(robot, link_idx)
    position, orientation = ret[0], pybullet.getEulerFromQuaternion(ret[1])
    return position, orientation
