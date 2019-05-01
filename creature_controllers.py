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

    def getNormalUpward(self):
        rootNode = self.skel.bodynodes[0]
        center = rootNode.to_world([0, 0, 0])
        offsetDirection = rootNode.to_world([0, 1, 0]) - center
        normal = offsetDirection / np.linalg.norm(offsetDirection)
        return normal

    def getNormalDot(self):
        normal = self.getNormalUpward()
        dot = np.dot(normal, np.array([0, 1, 0]))
        print(dot)
        return dot

    def getNormalEndpoints(self):
        rootNode = self.skel.bodynodes[0]
        center = rootNode.to_world([0, 0, 0])
        offsetDirection = rootNode.to_world([0, 5, 0])
        return center, offsetDirection


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

    def getNormalUpward(self):
        rootNode = self.skel.bodynodes[0]
        center = rootNode.to_world([0, 0, 0])
        offsetDirection = rootNode.to_world([0, 1, 0]) - center
        normal = offsetDirection / np.linalg.norm(offsetDirection)
        return normal

    def getNormalDot(self):
        normal = self.getNormalUpward()
        dot = np.dot(normal, np.array([0, 1, 0]))
        print(dot)
        return dot

    def getNormalEndpoints(self):
        rootNode = self.skel.bodynodes[0]
        center = rootNode.to_world([0, 0, 0])
        offsetDirection = rootNode.to_world([0, 5, 0])
        return center, offsetDirection

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

    def getNormalUpward(self):
        rootNode = self.skel.bodynodes[0]
        center = rootNode.to_world([0, 0, 0])
        offsetDirection = rootNode.to_world([0, 1, 0]) - center
        normal = offsetDirection / np.linalg.norm(offsetDirection)
        return normal

    def getNormalDot(self):
        normal = self.getNormalUpward()
        dot = np.dot(normal, np.array([0, 1, 0]))
        print(dot)
        return dot

    def getNormalEndpoints(self):
        rootNode = self.skel.bodynodes[0]
        center = rootNode.to_world([0, 0, 0])
        offsetDirection = rootNode.to_world([0, 5, 0])
        return center, offsetDirection

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

    def getNormalUpward(self):
        rootNode = self.skel.bodynodes[0]
        center = rootNode.to_world([0, 0, 0])
        offsetDirection = rootNode.to_world([0, 1, 0]) - center
        normal = offsetDirection / np.linalg.norm(offsetDirection)
        return normal

    def getNormalDot(self):
        normal = self.getNormalUpward()
        dot = np.dot(normal, np.array([0, 1, 0]))
        print(dot)
        return dot

    def getNormalEndpoints(self):
        rootNode = self.skel.bodynodes[0]
        center = rootNode.to_world([0, 0, 0])
        offsetDirection = rootNode.to_world([0, 5, 0])
        return center, offsetDirection

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

    def getNormalUpward(self):
        rootNode = self.skel.bodynodes[0]
        center = rootNode.to_world([0, 0, 0])
        offsetDirection = rootNode.to_world([0, 1, 0]) - center
        normal = offsetDirection / np.linalg.norm(offsetDirection)
        return normal

    def getNormalDot(self):
        normal = self.getNormalUpward()
        dot = np.dot(normal, np.array([0, 1, 0]))
        print(dot)
        return dot

    def getNormalEndpoints(self):
        rootNode = self.skel.bodynodes[0]
        center = rootNode.to_world([0, 0, 0])
        offsetDirection = rootNode.to_world([0, 5, 0])
        return center, offsetDirection


# vector: [backleg amplitude, backleg_phase, frontleg_amplitude, frontleg_phase, backshin_amplitude, backshin_phase,
# frontshin_amplitude, frontshin_phase]
class LeftRightSymmetricKneeQuadController(object):
    def __init__(self, skel, vector):
        self.skel = skel
        self.target = None
        self.Kp = np.array([0.0] * 6 + [900.0] * (self.skel.ndofs - 6))
        self.Kd = np.array([0.0] * 6 + [90.0] * (self.skel.ndofs - 6))
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
        pose[('leg2_joint')] = self.periodic(self.vector[0], (3*math.pi)/2, - 1 * self.vector[1])
        pose[('leg3_joint')] = self.periodic(self.vector[2], (3*math.pi)/2, self.vector[3])
        pose[('leg4_joint')] = self.periodic(self.vector[2], (3*math.pi)/2, -1 * self.vector[3])
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

    def getNormalUpward(self):
        rootNode = self.skel.bodynodes[0]
        center = rootNode.to_world([0, 0, 0])
        offsetDirection = rootNode.to_world([0, 1, 0]) - center
        normal = offsetDirection / np.linalg.norm(offsetDirection)
        return normal

    def getNormalDot(self):
        normal = self.getNormalUpward()
        dot = np.dot(normal, np.array([0, 1, 0]))
        print(dot)
        return dot

    def getNormalEndpoints(self):
        rootNode = self.skel.bodynodes[0]
        center = rootNode.to_world([0, 0, 0])
        offsetDirection = rootNode.to_world([0, 5, 0])
        return center, offsetDirection

# vector: [backleg amplitude, backleg_phase, frontleg_amplitude, frontleg_phase, backshin_amplitude, backshin_phase,
# frontshin_amplitude, frontshin_phase, phase_difference]
class LRSymmetricPhaseKneeQuadController(object):
    def __init__(self, skel, vector):
        self.skel = skel
        self.target = None
        self.Kp = np.array([0.0] * 6 + [900.0] * (self.skel.ndofs - 6))
        self.Kd = np.array([0.0] * 6 + [90.0] * (self.skel.ndofs - 6))
        self.vector = vector
        # rootNode = self.skel.bodynodes[0]
        # center = rootNode.to_world([0, 0, 0])
        # offsetPoint = rootNode.to_world([0, 1, 0]) - center
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

    def getNormalUpward(self):
        rootNode = self.skel.bodynodes[0]
        center = rootNode.to_world([0, 0, 0])
        offsetDirection = rootNode.to_world([0, 1, 0]) - center
        normal = offsetDirection / np.linalg.norm(offsetDirection)
        return normal

    def getNormalDot(self):
        normal = self.getNormalUpward()
        dot = np.dot(normal, np.array([0, 1, 0]))
        print(dot)
        return dot

    def getNormalEndpoints(self):
        rootNode = self.skel.bodynodes[0]
        center = rootNode.to_world([0, 0, 0])
        offsetDirection = rootNode.to_world([0, 5, 0])
        return center, offsetDirection

# vector: [backleg amplitude, backleg_phase, frontleg_amplitude, frontleg_phase, backshin_amplitude, backshin_phase,
# frontshin_amplitude, frontshin_phase]
class LeftRightSymmetricCapsuleQuadController(object):
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
        # if pose[4] < -.25:
        #     print("below")
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

    def getNormalUpward(self):
        rootNode = self.skel.bodynodes[0]
        center = rootNode.to_world([0, 0, 0])
        offsetDirection = rootNode.to_world([0, 1, 0]) - center
        normal = offsetDirection / np.linalg.norm(offsetDirection)
        return normal

    def getNormalDot(self):
        normal = self.getNormalUpward()
        dot = np.dot(normal, np.array([0, 1, 0]))
        print(dot)
        return dot

    def getNormalEndpoints(self):
        rootNode = self.skel.bodynodes[0]
        center = rootNode.to_world([0, 0, 0])
        offsetDirection = rootNode.to_world([0, 5, 0])
        return center, offsetDirection

# vector: [backleg amplitude, backleg_phase, frontleg_amplitude, frontleg_phase, backshin_amplitude, backshin_phase,
# frontshin_amplitude, frontshin_phase, backfoot_amplitude, backfoot_phase, frontfoot_amplitude, frontfoot_phase]
class LeftRightSymmetricWideFootQuadController(object):
    def __init__(self, skel, vector):
        self.skel = skel
        self.target = None
        self.Kp = np.array([0.0] * 6 + [1000.0] * (self.skel.ndofs - 6))
        self.Kd = np.array([0.0] * 6 + [100.0] * (self.skel.ndofs - 6))
        self.vector = vector
        # rootNode = self.skel.bodynodes[0]
        # center = rootNode.to_world([0, 0, 0])
        # offsetPoint = rootNode.to_world([0, 1, 0]) - center
        # print(center)
        # print(offsetPoint)

    def update_target_poses(self):
        pose = self.skel.q
        # if pose[4] < -.25:
        #     print("below")
        pose[('leg1_joint')] = self.periodic(self.vector[0], (3*math.pi)/2, self.vector[1])
        pose[('leg2_joint')] = self.periodic(self.vector[0], (3*math.pi)/2, self.vector[1] - math.pi/2)
        pose[('leg3_joint')] = self.periodic(self.vector[2], (3*math.pi)/2, self.vector[3])
        pose[('leg4_joint')] = self.periodic(self.vector[2], (3*math.pi)/2, self.vector[3] - math.pi/2)
        pose[('shin1_joint')] = self.periodic(self.vector[4], (3*math.pi)/2, self.vector[5])
        pose[('shin2_joint')] = self.periodic(self.vector[4], (3*math.pi)/2, self.vector[5])
        pose[('shin3_joint')] = self.periodic(self.vector[6], (3*math.pi)/2, self.vector[7])
        pose[('shin4_joint')] = self.periodic(self.vector[6], (3*math.pi)/2, self.vector[7])
        pose[('foot1_joint')] = self.periodic(self.vector[8]*(.3/(math.pi/4)), (3*math.pi)/2, self.vector[9])
        pose[('foot2_joint')] = self.periodic(self.vector[8]*(.3/(math.pi/4)), (3*math.pi)/2, self.vector[9] - math.pi/2)
        pose[('foot3_joint')] = self.periodic(self.vector[10]*(.3/(math.pi/4)), (3*math.pi)/2, self.vector[11])
        pose[('foot4_joint')] = self.periodic(self.vector[10]*(.3/(math.pi/4)), (3*math.pi)/2, self.vector[11] - math.pi/2)
        return pose

    def periodic(self, amplitude, period, phase):
        return (amplitude * np.sin(period * self.skel.world.t + phase))

    def compute(self):
        self.target = self.update_target_poses()
        return -self.Kp * (self.skel.q - self.target) - self.Kd * self.skel.dq

    def getNormalUpward(self):
        rootNode = self.skel.bodynodes[0]
        center = rootNode.to_world([0, 0, 0])
        offsetDirection = rootNode.to_world([0, 1, 0]) - center
        normal = offsetDirection / np.linalg.norm(offsetDirection)
        return normal

    def getNormalDot(self):
        normal = self.getNormalUpward()
        dot = np.dot(normal, np.array([0, 1, 0]))
        print(dot)
        return dot

    def getNormalEndpoints(self):
        rootNode = self.skel.bodynodes[0]
        center = rootNode.to_world([0, 0, 0])
        offsetDirection = rootNode.to_world([0, 5, 0])
        return center, offsetDirection

# vector: [backleg_amplitude, backleg_phase, frontleg_amplitude, frontleg_phase, backshin_amplitude, backshin_phase,
# frontshin_amplitude, frontshin_phase]
class SymmetricKneeQuadController(object):
    def __init__(self, skel, vector):
        self.skel = skel
        self.target = None
        self.Kp = np.array([0.0] * 6 + [400.0] * (self.skel.ndofs - 6))
        self.Kd = np.array([0.0] * 6 + [40.0] * (self.skel.ndofs - 6))
        self.vector = vector
        # rootNode = self.skel.bodynodes[0]
        # center = rootNode.to_world([0, 0, 0])
        # offsetPoint = rootNode.to_world([0, 1, 0]) - center
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
        self.target = self.update_target_poses()
        return -self.Kp * (self.skel.q - self.target) - self.Kd * self.skel.dq

    def getNormalUpward(self):
        rootNode = self.skel.bodynodes[0]
        center = rootNode.to_world([0, 0, 0])
        offsetDirection = rootNode.to_world([0, 1, 0]) - center
        normal = offsetDirection / np.linalg.norm(offsetDirection)
        return normal

    def getNormalDot(self):
        normal = self.getNormalUpward()
        dot = np.dot(normal, np.array([0, 1, 0]))
        print(dot)
        return dot

    def getNormalEndpoints(self):
        rootNode = self.skel.bodynodes[0]
        center = rootNode.to_world([0, 0, 0])
        offsetDirection = rootNode.to_world([0, 5, 0])
        return center, offsetDirection


# vector:  [row1BodyY_amp, row1BodyY_phase, row1BodyX_amp, row1BodyX_phase, row1Mid_amp, row1Mid_phase, row1End_amp, row1End_phase,
#           row2BodyY_amp, row2BodyY_phase, row2BodyX_amp, row2BodyX_phase, row2Mid_amp, row2Mid_phase, row2End_amp, row2End_phase,
#           row3BodyY_amp, row3BodyY_phase, row3BodyX_amp, row3BodyX_phase, row3Mid_amp, row3Mid_phase, row3End_amp, row3End_phase,
#           row4BodyY_amp, row4BodyY_phase, row4BodyX_amp, row4BodyX_phase, row4Mid_amp, row4Mid_phase, row4End_amp, row4End_phase]
class SpiderController(object):
    def __init__(self, skel, vector):
        self.skel = skel
        self.target = None
        self.Kp = np.array([0.0] * 6 + [400.0] * (self.skel.ndofs - 6))
        self.Kd = np.array([0.0] * 6 + [40.0] * (self.skel.ndofs - 6))
        self.vector = vector

    def update_target_poses(self):
        pose = self.skel.q

        pose[('leg1_joint_1')] = self.periodic(self.vector[0], math.pi / 2, self.vector[1])
        pose[('leg1_joint_2')] = self.periodic(self.vector[2], math.pi / 2, self.vector[3])
        pose[('leg1Mid_joint')] = self.periodic(self.vector[4], math.pi / 2, self.vector[5])
        pose[('leg1End_joint')] = self.periodic(self.vector[6], math.pi / 2, self.vector[7])

        pose[('leg2_joint_1')] = self.periodic(self.vector[0], math.pi / 2, self.vector[1])
        pose[('leg2_joint_2')] = self.periodic(self.vector[2], math.pi / 2, self.vector[3])
        pose[('leg2Mid_joint')] = self.periodic(self.vector[4], math.pi / 2, self.vector[5])
        pose[('leg2End_joint')] = self.periodic(self.vector[6], math.pi / 2, self.vector[7])

        pose[('leg3_joint_1')] = self.periodic(self.vector[8], math.pi / 2, self.vector[9])
        pose[('leg3_joint_2')] = self.periodic(self.vector[10], math.pi / 2, self.vector[11])
        pose[('leg3Mid_joint')] = self.periodic(self.vector[12], math.pi / 2, self.vector[13])
        pose[('leg3End_joint')] = self.periodic(self.vector[14], math.pi / 2, self.vector[15])

        pose[('leg4_joint_1')] = self.periodic(self.vector[8], math.pi / 2, self.vector[9])
        pose[('leg4_joint_2')] = self.periodic(self.vector[10], math.pi / 2, self.vector[11])
        pose[('leg4Mid_joint')] = self.periodic(self.vector[12], math.pi / 2, self.vector[13])
        pose[('leg4End_joint')] = self.periodic(self.vector[14], math.pi / 2, self.vector[15])

        pose[('leg5_joint_1')] = self.periodic(self.vector[16], math.pi / 2, self.vector[17])
        pose[('leg5_joint_2')] = self.periodic(self.vector[18], math.pi / 2, self.vector[19])
        pose[('leg5Mid_joint')] = self.periodic(self.vector[20], math.pi / 2, self.vector[21])
        pose[('leg5End_joint')] = self.periodic(self.vector[22], math.pi / 2, self.vector[23])

        pose[('leg6_joint_1')] = self.periodic(self.vector[16], math.pi / 2, self.vector[17])
        pose[('leg6_joint_2')] = self.periodic(self.vector[18], math.pi / 2, self.vector[19])
        pose[('leg6Mid_joint')] = self.periodic(self.vector[20], math.pi / 2, self.vector[21])
        pose[('leg6End_joint')] = self.periodic(self.vector[22], math.pi / 2, self.vector[23])

        pose[('leg7_joint_1')] = self.periodic(self.vector[24], math.pi / 2, self.vector[25])
        pose[('leg7_joint_2')] = self.periodic(self.vector[26], math.pi / 2, self.vector[27])
        pose[('leg7Mid_joint')] = self.periodic(self.vector[28], math.pi / 2, self.vector[29])
        pose[('leg7End_joint')] = self.periodic(self.vector[30], math.pi / 2, self.vector[31])

        pose[('leg8_joint_1')] = self.periodic(self.vector[24], math.pi / 2, self.vector[25])
        pose[('leg8_joint_2')] = self.periodic(self.vector[26], math.pi / 2, self.vector[27])
        pose[('leg8Mid_joint')] = self.periodic(self.vector[28], math.pi / 2, self.vector[29])
        pose[('leg8End_joint')] = self.periodic(self.vector[30], math.pi / 2, self.vector[31])

        return pose

    def periodic(self, amplitude, period, phase):
        return (amplitude * np.sin(period * self.skel.world.t + phase))

    def compute(self):
        self.target = self.update_target_poses()
        self.getNormalDot()
        # print("center: ", center)
        # print("offset: ", offsetDirection)
        return -self.Kp * (self.skel.q - self.target) - self.Kd * self.skel.dq

    def getNormalUpward(self):
        rootNode = self.skel.bodynodes[0]
        center = rootNode.to_world([0, 0, 0])
        offsetDirection = rootNode.to_world([0, 1, 0]) - center
        normal = offsetDirection / np.linalg.norm(offsetDirection)
        return normal

    def getNormalDot(self):
        normal = self.getNormalUpward()
        dot = np.dot(normal, np.array([0, 1, 0]))
        print(dot, np.arccos(dot))
        return dot

    def getNormalEndpoints(self):
        rootNode = self.skel.bodynodes[0]
        center = rootNode.to_world([0, 0, 0])
        offsetDirection = rootNode.to_world([0, 5, 0])
        return center, offsetDirection




class BasicSpiderController(object):
    def __init__(self, skel, vector):
        self.skel = skel
        self.target = None
        self.Kp = np.array([0.0] * 6 + [400.0] * (self.skel.ndofs - 6))
        self.Kd = np.array([0.0] * 6 + [40.0] * (self.skel.ndofs - 6))
        self.vector = vector

    def update_target_poses(self):
        pose = self.skel.q

        pose[('leg1_joint_1')] = self.periodic(self.vector[0], math.pi / 2, self.vector[1])
        pose[('leg1_joint_2')] = self.periodic(self.vector[2], math.pi / 2, self.vector[3])
        # pose[('leg1Mid_joint')] = self.periodic(self.vector[4], math.pi / 2, self.vector[5])
        # pose[('leg1End_joint')] = self.periodic(self.vector[6], math.pi / 2, self.vector[7])

        pose[('leg2_joint_1')] = self.periodic(self.vector[0], math.pi / 2, self.vector[1])
        pose[('leg2_joint_2')] = self.periodic(self.vector[2], math.pi / 2, self.vector[3])
        # pose[('leg2Mid_joint')] = self.periodic(self.vector[4], math.pi / 2, self.vector[5])
        # pose[('leg2End_joint')] = self.periodic(self.vector[6], math.pi / 2, self.vector[7])

        pose[('leg3_joint_1')] = self.periodic(self.vector[4], math.pi / 2, self.vector[5])
        pose[('leg3_joint_2')] = self.periodic(self.vector[6], math.pi / 2, self.vector[7])
        # pose[('leg3Mid_joint')] = self.periodic(self.vector[12], math.pi / 2, self.vector[13])
        # pose[('leg3End_joint')] = self.periodic(self.vector[14], math.pi / 2, self.vector[15])

        pose[('leg4_joint_1')] = self.periodic(self.vector[4], math.pi / 2, self.vector[5])
        pose[('leg4_joint_2')] = self.periodic(self.vector[6], math.pi / 2, self.vector[7])
        # pose[('leg4Mid_joint')] = self.periodic(self.vector[12], math.pi / 2, self.vector[13])
        # pose[('leg4End_joint')] = self.periodic(self.vector[14], math.pi / 2, self.vector[15])

        pose[('leg5_joint_1')] = self.periodic(self.vector[8], math.pi / 2, self.vector[9])
        pose[('leg5_joint_2')] = self.periodic(self.vector[10], math.pi / 2, self.vector[11])
        # pose[('leg5Mid_joint')] = self.periodic(self.vector[20], math.pi / 2, self.vector[21])
        # pose[('leg5End_joint')] = self.periodic(self.vector[22], math.pi / 2, self.vector[23])

        pose[('leg6_joint_1')] = self.periodic(self.vector[8], math.pi / 2, self.vector[9])
        pose[('leg6_joint_2')] = self.periodic(self.vector[10], math.pi / 2, self.vector[11])
        # pose[('leg6Mid_joint')] = self.periodic(self.vector[20], math.pi / 2, self.vector[21])
        # pose[('leg6End_joint')] = self.periodic(self.vector[22], math.pi / 2, self.vector[23])

        pose[('leg7_joint_1')] = self.periodic(self.vector[12], math.pi / 2, self.vector[13])
        pose[('leg7_joint_2')] = self.periodic(self.vector[14], math.pi / 2, self.vector[15])
        # pose[('leg7Mid_joint')] = self.periodic(self.vector[28], math.pi / 2, self.vector[29])
        # pose[('leg7End_joint')] = self.periodic(self.vector[30], math.pi / 2, self.vector[31])

        pose[('leg8_joint_1')] = self.periodic(self.vector[12], math.pi / 2, self.vector[13])
        pose[('leg8_joint_2')] = self.periodic(self.vector[14], math.pi / 2, self.vector[15])
        # pose[('leg8Mid_joint')] = self.periodic(self.vector[28], math.pi / 2, self.vector[29])
        # pose[('leg8End_joint')] = self.periodic(self.vector[30], math.pi / 2, self.vector[31])

        return pose

    def periodic(self, amplitude, period, phase):
        return (amplitude * np.sin(period * self.skel.world.t + phase))

    def compute(self):
        self.target = self.update_target_poses()
        return -self.Kp * (self.skel.q - self.target) - self.Kd * self.skel.dq

    def getNormalUpward(self):
        rootNode = self.skel.bodynodes[0]
        center = rootNode.to_world([0, 0, 0])
        offsetDirection = rootNode.to_world([0, 1, 0]) - center
        normal = offsetDirection / np.linalg.norm(offsetDirection)
        return normal

    def getNormalDot(self):
        normal = self.getNormalUpward()
        dot = np.dot(normal, np.array([0, 1, 0]))
        # print(dot, np.arccos(dot))
        # print(np.arccos(dot))
        return dot

    def getNormalEndpoints(self):
        rootNode = self.skel.bodynodes[0]
        center = rootNode.to_world([0, 0, 0])
        offsetDirection = rootNode.to_world([0, 5, 0])
        return center, offsetDirection