"""
Class to control Kuka robot
"""
import sys
import os
import pybullet
import pybullet_data

SCRIPT_DIR = os.path.dirname(os.path.realpath(__file__))
sys.path.append(os.path.join(SCRIPT_DIR, '..'))
from blue import Robot, getJointRanges

class Kuka(Robot):
    LINK_IDX = 6

    def __init__(self, robot_path):
        flags = pybullet.URDF_USE_SELF_COLLISION | pybullet.URDF_USE_SELF_COLLISION_EXCLUDE_ALL_PARENTS | pybullet.URDF_MERGE_FIXED_LINKS | pybullet.URDF_USE_INERTIA_FROM_FILE
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
            self.id, self.moving_joints_idx, pybullet.POSITION_CONTROL,
            targetPositions=target_positions,
            )

    def debug_arm_idx(self):
        # press 'w' to see this links highlighted
        pybullet.setDebugObjectColor(self.id, self.LINK_IDX, [1, 0, 0])
