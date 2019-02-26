import pydart2 as pydart
import numpy as np
import math

# vector: [dPhase, amplitude, period]
class ThreeVarController(object):
    def __init__(self, skel, vector):
        self.skel = skel
        self.target = None
        self.Kp = np.array([0.0] * 6 + [400.0] * (self.skel.ndofs - 6))
        self.Kd = np.array([0.0] * 6 + [40.0] * (self.skel.ndofs - 6))
        self.dPhase = vector[0]
        self.amp = vector[1]
        self.period = vector[2]
        #add bounds on angles
        self.minJointAngle = -math.pi/2
        self.maxJointAngle = math.pi/2

    def update_target_poses(self):
        skel = self.skel
        pose = self.skel.q
        pose[('joint_2', 'joint_3', 'joint_4', 'joint_5')] = self.periodic(self.amp, self.period, 0), self.periodic(self.amp, self.period, 1*self.dPhase),\
                                                             self.periodic(self.amp, self.period, 2*self.dPhase), self.periodic(self.amp, self.period, 3*self.dPhase)
        return pose

    def periodic(self, amplitude, period, phase):
        return (amplitude * np.sin(period * self.skel.world.t + phase))


    def compute(self):
        self.target = self.update_target_poses()
        return -self.Kp * (self.skel.q - self.target) - self.Kd * self.skel.dq


# vector: [joint1Amp, joint2Amp, joint3Amp, joint4Amp, joint1Phase, joint2Phase, joint3Phase, joint4Phase, period]
class NineVarController(object):
    def __init__(self, skel, vector):
        self.skel = skel
        self.target = None
        self.Kp = np.array([0.0] * 6 + [400.0] * (self.skel.ndofs - 6))
        self.Kd = np.array([0.0] * 6 + [40.0] * (self.skel.ndofs - 6))
        self.inputVector = vector
        #add bounds on angles
        self.minJointAngle = -math.pi/2
        self.maxJointAngle = math.pi/2

    def update_target_poses(self):
        skel = self.skel
        pose = self.skel.q
        pose[('joint_2', 'joint_3', 'joint_4', 'joint_5')] = self.periodic(self.inputVector[0], self.inputVector[8], self.inputVector[4]), self.periodic(self.inputVector[1], self.inputVector[8], self.inputVector[5]),\
                                                             self.periodic(self.inputVector[2], self.inputVector[8], self.inputVector[6]), self.periodic(self.inputVector[3], self.inputVector[8], self.inputVector[7])
        return pose

    def periodic(self, amplitude, period, phase):
        return (amplitude * np.sin(period * self.skel.world.t + phase))


    def compute(self):
        self.target = self.update_target_poses()
        return -self.Kp * (self.skel.q - self.target) - self.Kd * self.skel.dq

# vector: [leg1_amplitude, leg1_phase, leg2_amplitude, leg2_phase, leg3_amplitude, leg3_phase, leg4_amplitude, leg4_phase]
class SimpleQuadController(object):
    def __init__(self, skel, vector):
        self.skel = skel
        self.target = None
        self.Kp = np.array([0.0] * 6 + [400.0] * (self.skel.ndofs - 6))
        self.Kd = np.array([0.0] * 6 + [40.0] * (self.skel.ndofs - 6))
        # self.amplitude = vector[0]
        # self.period = vector[1]
        # self.phase = vector[2]
        self.vector = vector


    def update_target_poses(self):
        pose = self.skel.q
        pose[('leg1_joint')] = self.periodic(self.vector[0], math.pi/2, self.vector[1])
        pose[('leg2_joint')] = self.periodic(self.vector[2], math.pi/2, self.vector[3])
        pose[('leg3_joint')] = self.periodic(self.vector[4], math.pi/2, self.vector[5])
        pose[('leg4_joint')] = self.periodic(self.vector[6], math.pi/2, self.vector[7])
        return pose

    def periodic(self, amplitude, period, phase):
        return (amplitude * np.sin(period * self.skel.world.t + phase))


    def compute(self):
        self.target = self.update_target_poses()
        return -self.Kp * (self.skel.q - self.target) - self.Kd * self.skel.dq

# vector: [leg1_amplitude, leg1_phase, leg2_amplitude, leg2_phase, leg3_amplitude, leg3_phase, leg4_amplitude, leg4_phase,
# shin1_amplitude, shin1_phase, shin2_amplitude, shin2_phase, shin3_amplitude, shin3_phase, shin4_amplitude, shin4_phase]
class KneeQuadController(object):
    def __init__(self, skel, vector):
        self.skel = skel
        self.target = None
        self.Kp = np.array([0.0] * 6 + [400.0] * (self.skel.ndofs - 6))
        self.Kd = np.array([0.0] * 6 + [40.0] * (self.skel.ndofs - 6))
        self.vector = vector

    def update_target_poses(self):
        pose = self.skel.q
        pose[('leg1_joint')] = self.periodic(self.vector[0], math.pi / 2, self.vector[1])
        pose[('leg2_joint')] = self.periodic(self.vector[2], math.pi / 2, self.vector[3])
        pose[('leg3_joint')] = self.periodic(self.vector[4], math.pi / 2, self.vector[5])
        pose[('leg4_joint')] = self.periodic(self.vector[6], math.pi / 2, self.vector[7])
        pose[('shin1_joint')] = self.periodic(self.vector[8], math.pi / 2, self.vector[9])
        pose[('shin2_joint')] = self.periodic(self.vector[10], math.pi / 2, self.vector[11])
        pose[('shin3_joint')] = self.periodic(self.vector[12], math.pi / 2, self.vector[13])
        pose[('shin4_joint')] = self.periodic(self.vector[14], math.pi / 2, self.vector[15])
        return pose

    def periodic(self, amplitude, period, phase):
        return (amplitude * np.sin(period * self.skel.world.t + phase))

    def compute(self):
        self.target = self.update_target_poses()
        return -self.Kp * (self.skel.q - self.target) - self.Kd * self.skel.dq

# vector: [backleg_amplitude, backleg_phase, frontleg_amplitude, frontleg_phase, backshin_amplitude, backshin_phase,
# frontshin_amplitude, frontshin_phase]
class SymmetricKneeQuadController(object):
    def __init__(self, skel, vector):
        self.skel = skel
        self.target = None
        self.Kp = np.array([0.0] * 6 + [400.0] * (self.skel.ndofs - 6))
        self.Kd = np.array([0.0] * 6 + [40.0] * (self.skel.ndofs - 6))
        self.vector = vector
        rootNode = self.skel.bodynodes[0]
        center = rootNode.to_world([0, 0, 0])
        offsetPoint = rootNode.to_world([0, 1, 0]) - center
        # print(center)
        # print(offsetPoint)

    def update_target_poses(self):
        pose = self.skel.q
        pose[('leg1_joint')] = self.periodic(self.vector[0], math.pi / 2, self.vector[1])
        pose[('leg2_joint')] = self.periodic(self.vector[0], math.pi / 2, self.vector[1])
        pose[('leg3_joint')] = self.periodic(self.vector[2], math.pi / 2, self.vector[3])
        pose[('leg4_joint')] = self.periodic(self.vector[2], math.pi / 2, self.vector[3])
        pose[('shin1_joint')] = self.periodic(self.vector[4], math.pi / 2, self.vector[5])
        pose[('shin2_joint')] = self.periodic(self.vector[4], math.pi / 2, self.vector[5])
        pose[('shin3_joint')] = self.periodic(self.vector[6], math.pi / 2, self.vector[7])
        pose[('shin4_joint')] = self.periodic(self.vector[6], math.pi / 2, self.vector[7])
        return pose

    def periodic(self, amplitude, period, phase):
        return (amplitude * np.sin(period * self.skel.world.t + phase))

    def compute(self):
        rootNode = self.skel.bodynodes[0]
        center = rootNode.to_world([0, 0, 0])
        offsetDirection = rootNode.to_world([0, 1, 0]) - center
        offsetDirection = offsetDirection/np.linalg.norm(offsetDirection)
        # print("center: ", center)
        # print("offset: ", offsetDirection)
        self.target = self.update_target_poses()
        return -self.Kp * (self.skel.q - self.target) - self.Kd * self.skel.dq


# vector: [backleg amplitude, backleg_phase, frontleg_amplitude, frontleg_phase, backshin_amplitude, backshin_phase,
# frontshin_amplitude, frontshin_phase]
class LeftRightSymmetricKneeQuadController(object):
    def __init__(self, skel, vector):
        self.skel = skel
        self.target = None
        self.Kp = np.array([0.0] * 6 + [400.0] * (self.skel.ndofs - 6))
        self.Kd = np.array([0.0] * 6 + [40.0] * (self.skel.ndofs - 6))
        self.vector = vector
        rootNode = self.skel.bodynodes[0]
        center = rootNode.to_world([0, 0, 0])
        offsetPoint = rootNode.to_world([0, 1, 0]) - center
        # print(center)
        # print(offsetPoint)

    def update_target_poses(self):
        pose = self.skel.q
        if pose[4] < -.25:
            print("below")
        pose[('leg1_joint')] = self.periodic(self.vector[0], (3*math.pi)/2, self.vector[1])
        pose[('leg2_joint')] = self.periodic(self.vector[0], (3*math.pi)/2, self.vector[1] - math.pi/2)
        pose[('leg3_joint')] = self.periodic(self.vector[2], (3*math.pi)/2, self.vector[3])
        pose[('leg4_joint')] = self.periodic(self.vector[2], (3*math.pi)/2, self.vector[3] - math.pi/2)
        pose[('shin1_joint')] = self.periodic(self.vector[4], (3*math.pi)/2, self.vector[5])
        pose[('shin2_joint')] = self.periodic(self.vector[4], (3*math.pi)/2, self.vector[5])
        pose[('shin3_joint')] = self.periodic(self.vector[6], (3*math.pi)/2, self.vector[7])
        pose[('shin4_joint')] = self.periodic(self.vector[6], (3*math.pi)/2, self.vector[7])
        return pose

    def periodic(self, amplitude, period, phase):
        return (amplitude * np.sin(period * self.skel.world.t + phase))

    def compute(self):
        rootNode = self.skel.bodynodes[0]
        center = rootNode.to_world([0, 0, 0])
        offsetDirection = rootNode.to_world([0, 1, 0]) - center
        offsetDirection = offsetDirection/np.linalg.norm(offsetDirection)
        # print("center: ", center)
        # print("offset: ", offsetDirection)
        self.target = self.update_target_poses()
        return -self.Kp * (self.skel.q - self.target) - self.Kd * self.skel.dq

# vector: [backleg amplitude, backleg_phase, frontleg_amplitude, frontleg_phase, backshin_amplitude, backshin_phase,
# frontshin_amplitude, frontshin_phase, phase_difference]
class LRSymmetricPhaseKneeQuadController(object):
    def __init__(self, skel, vector):
        self.skel = skel
        self.target = None
        self.Kp = np.array([0.0] * 6 + [400.0] * (self.skel.ndofs - 6))
        self.Kd = np.array([0.0] * 6 + [40.0] * (self.skel.ndofs - 6))
        self.vector = vector
        rootNode = self.skel.bodynodes[0]
        center = rootNode.to_world([0, 0, 0])
        offsetPoint = rootNode.to_world([0, 1, 0]) - center
        # print(center)
        # print(offsetPoint)

    def update_target_poses(self):
        pose = self.skel.q
        pose[('leg1_joint')] = self.periodic(self.vector[0], (3*math.pi)/2, self.vector[1])
        pose[('leg2_joint')] = self.periodic(self.vector[0], (3*math.pi)/2, self.vector[1] - self.vector[8])
        pose[('leg3_joint')] = self.periodic(self.vector[2], (3*math.pi)/2, self.vector[3])
        pose[('leg4_joint')] = self.periodic(self.vector[2], (3*math.pi)/2, self.vector[3] - self.vector[8])
        pose[('shin1_joint')] = self.periodic(self.vector[4], (3*math.pi)/2, self.vector[5])
        pose[('shin2_joint')] = self.periodic(self.vector[4], (3*math.pi)/2, self.vector[5])
        pose[('shin3_joint')] = self.periodic(self.vector[6], (3*math.pi)/2, self.vector[7])
        pose[('shin4_joint')] = self.periodic(self.vector[6], (3*math.pi)/2, self.vector[7])
        return pose

    def periodic(self, amplitude, period, phase):
        return (amplitude * np.sin(period * self.skel.world.t + phase))

    def compute(self):
        rootNode = self.skel.bodynodes[0]
        center = rootNode.to_world([0, 0, 0])
        offsetDirection = rootNode.to_world([0, 1, 0]) - center
        offsetDirection = offsetDirection/np.linalg.norm(offsetDirection)
        # print("center: ", center)
        # print("offset: ", offsetDirection)
        self.target = self.update_target_poses()
        return -self.Kp * (self.skel.q - self.target) - self.Kd * self.skel.dq