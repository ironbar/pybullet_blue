"""
Utilities to control Blue robot
"""
import os
import time
from tqdm import tqdm
import pybullet

from utils import debug_position

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

    def startup(self, is_real_time=True):
        for _ in tqdm(range(10), desc='startup'):
            self.go_to_rest_pose()
            if is_real_time:
                time.sleep(0.01)
            else:
                pybullet.stepSimulation()

    def get_mass(self):
        link_mass = [pybullet.getDynamicsInfo(self.id, idx)[0] for idx in range(pybullet.getNumJoints(self.id))]
        print('Total mass is : %.1f (%s)' % (sum(link_mass), link_mass))


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

    def get_right_arm_pose(self):
        return self._get_link_state(self.RIGHT_ARM_LINK_IDX)

    def get_left_arm_pose(self):
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
            targetPositions=[1, -0.7]*2)

    def open_right_clamp(self):
        pybullet.setJointMotorControlArray(
            self.id, self.moving_joints_idx[8:12], pybullet.POSITION_CONTROL,
            targetPositions=[0]*4)

    def close_left_clamp(self):
        pybullet.setJointMotorControlArray(
            self.id, self.moving_joints_idx[20:24], pybullet.POSITION_CONTROL,
            targetPositions=[1, -0.7]*2)

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

    def control(self, left_pose, right_pose, left_clamp, right_clamp, do_debug_position=False):
        """
        Simpler interface for moving the arms and controlling the clamps
        """
        self.move_right_arm(*right_pose)
        if do_debug_position: debug_position(right_pose[0], self.get_right_arm_pose()[0])

        self.move_left_arm(*left_pose)
        if do_debug_position: debug_position(left_pose[0], self.get_left_arm_pose()[0])

        if right_clamp:
            self.close_right_clamp()
        else:
            self.open_right_clamp()
        if left_clamp:
            self.close_left_clamp()
        else:
            self.open_left_clamp()


class BlueArm(Robot):
    LINK_IDX = 6

    def __init__(self, robot_path, base_position=None):
        flags = pybullet.URDF_USE_SELF_COLLISION | pybullet.URDF_USE_SELF_COLLISION_EXCLUDE_ALL_PARENTS | pybullet.URDF_MERGE_FIXED_LINKS | pybullet.URDF_USE_INERTIA_FROM_FILE
        base_position = base_position or [0, 0, 0]
        self.id = pybullet.loadURDF(robot_path, base_position, useFixedBase=1, flags=flags)
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
            targetPositions=[1, -0.7]*2)

    def open_clamp(self):
        pybullet.setJointMotorControlArray(
            self.id, self.moving_joints_idx[8:12], pybullet.POSITION_CONTROL,
            targetPositions=[0]*4)

    def debug_arm_idx(self):
        # press 'w' to see this links highlighted
        pybullet.setDebugObjectColor(self.id, self.LINK_IDX, [1, 0, 0])


class BlueRobotV2():
    def __init__(self):
        robots_dir = os.path.join(
            os.path.dirname(os.path.realpath(__file__)), 'robots')
        self.left_arm = BlueArm(os.path.join(robots_dir, 'blue_left_v2.urdf'), [0, 0.3, 1])
        self.right_arm = BlueArm(os.path.join(robots_dir, 'blue_right_v2.urdf'), [0, -0.3, 1])
        pybullet.loadURDF(os.path.join(robots_dir, 'blue_base_v2.urdf'),
                          [0, 0, 1], useFixedBase=1)

    def control(self, left_pose, right_pose, left_clamp, right_clamp, do_debug_position=False):
        """
        Simpler interface for moving the arms and controlling the clamps
        """
        self.left_arm.move(*left_pose)
        if do_debug_position: debug_position(left_pose[0], self.left_arm.get_pose()[0])
        self.right_arm.move(*right_pose)
        if do_debug_position: debug_position(right_pose[0], self.right_arm.get_pose()[0])
        if right_clamp:
            self.right_arm.close_clamp()
        else:
            self.right_arm.open_clamp()
        if left_clamp:
            self.left_arm.close_clamp()
        else:
            self.left_arm.open_clamp()

    def startup(self):
        self.left_arm.startup()
        self.right_arm.startup()

    def debug_arm_idx(self):
        self.right_arm.debug_arm_idx()
        self.left_arm.debug_arm_idx()

    def get_left_arm_pose(self):
        return self.left_arm.get_pose()

    def get_right_arm_pose(self):
        return self.right_arm.get_pose()


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
            rp = pybullet.getJointState(bodyId, i)[0]
            # Instead of that I will define a better rest position
            # This has clearly improved the Inverse kinematics calculation
            # However on v2 version of blue I have increased the range of the motors and probably
            # it is not necessary anymore. Or I may find a better rest position.
            # if ul == 0:
            #     rp = ul - jr/4
            # elif ul == -1:
            #     rp = 0
            # else:
            #     rp = ll + jr/2

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
