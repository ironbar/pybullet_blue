"""
"""
import pybullet
import pybullet_data

class PoseControl():
    """
    Sliders for controlling the pose of the robot
    """
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

    def get_pose(self):
        orientation = [pybullet.readUserDebugParameter(idx) for idx in self.orientation_idx]
        position = [pybullet.readUserDebugParameter(idx) for idx in self.position_idx]
        return position, orientation

class ClampControl():
    """
    Button for controlling the clamp of the robot
    """
    def __init__(self, prefix):
        self.id = pybullet.addUserDebugParameter('%s clamp' % prefix, 0, -1, 0)
        self.value = 0

    def close_clamp(self):
        return pybullet.readUserDebugParameter(self.id) % 2

def debug_position(goal, source, lifetime=0.5):
    pybullet.addUserDebugLine(
        goal, source, lineColorRGB=[1, 0, 0], lifeTime=lifetime, lineWidth=2)

def set_minimal_environment():
    pybullet.setAdditionalSearchPath(pybullet_data.getDataPath())
    pybullet.loadURDF("plane.urdf")
    pybullet.setGravity(0, 0, -9.81)
    pybullet.setRealTimeSimulation(1)
