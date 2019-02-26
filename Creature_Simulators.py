import pydart2 as pydart
import creature_controllers
import math
import time
import numpy as np

class ThreeVarSimulator(pydart.World):
    numVars = 3
    string = "/Results/ThreeVarWorm/ThreeVarInchWorm"
    def __init__(self, inputVector):
        super(ThreeVarSimulator, self).__init__(0.0003, './data/skel/longWorm.skel')
        self.controller = creature_controllers.ThreeVarController(self.skeletons[1], inputVector)
        self.skeletons[1].set_controller(self.controller)


class NineVarSimulator(pydart.World):
    numVars = 9
    string = "/Results/NineVarWorm/NineVarInchWorm"
    def __init__(self, inputVector):
        super(NineVarSimulator, self).__init__(0.0003, './data/skel/longWorm.skel')
        self.controller = creature_controllers.NineVarController(self.skeletons[1], inputVector)
        self.skeletons[1].set_controller(self.controller)


class SimpleQuadSimulator(pydart.World):
    lb = [-math.pi / 2, -math.pi, -math.pi / 2, -math.pi, -math.pi / 2, -math.pi, -math.pi / 2, -math.pi]
    hb = [math.pi / 2, math.pi, math.pi / 2, math.pi, math.pi / 2, math.pi, math.pi / 2, math.pi]
    numVars = 8
    string = "/Results/SimpleQuad/SimpleQuad"
    def __init__(self, inputVector):
        super(SimpleQuadSimulator, self).__init__(0.0003, './data/skel/quadriped.skel')
        self.controller = creature_controllers.SimpleQuadController(self.skeletons[1], inputVector)
        self.skeletons[1].set_controller(self.controller)


class KneeQuadSimulator(pydart.World):
    lb = [-math.pi / 2, -math.pi, -math.pi / 2, -math.pi, -math.pi / 2, -math.pi, -math.pi / 2, -math.pi] * 2
    hb = [math.pi / 2, math.pi, math.pi / 2, math.pi, math.pi / 2, math.pi, math.pi / 2, math.pi] * 2
    numVars = 16
    string = "/Results/KneeQuad/KneeQuad/"
    def __init__(self, inputVector):
        super(KneeQuadSimulator, self).__init__(0.0001, './data/skel/knee_quadriped.skel')
        self.controller = creature_controllers.KneeQuadController(self.skeletons[1], inputVector)
        self.skeletons[1].set_controller(self.controller)

class SymmetricKneeQuadSimulator(pydart.World):
    lb = [-math.pi / 4, -math.pi / 4, -math.pi / 4, -math.pi/ 4, -math.pi / 4, -math.pi / 4, -math.pi / 4, -math.pi / 4]
    hb = [math.pi / 4, math.pi / 4, math.pi / 4, math.pi / 4 , math.pi / 4, math.pi / 4, math.pi / 4, math.pi / 4]
    numVars = 8
    simulatorType = "SymmetricKneeQuad"
    string = "/Results/SymmetricKneeQuad/"
    def __init__(self, inputVector):
        super(SymmetricKneeQuadSimulator, self).__init__(0.0001, './data/skel/knee_quadriped.skel')
        self.controller = creature_controllers.SymmetricKneeQuadController(self.skeletons[1], inputVector)
        self.skeletons[1].set_controller(self.controller)
    def getX0(self):
        x0 = np.random.uniform(low=-math.pi / 4, high=math.pi / 4, size=(self.numVars,))
        # x0 = np.append(x0Short, (np.random.uniform(low = 0, high=2*math.pi, size = 1)))
        return x0

class LeftRightSymmetricKneeQuadSimulator(pydart.World):
    lb = [-math.pi / 4, -math.pi / 4, -math.pi / 4, -math.pi/ 4, -math.pi / 4, -math.pi / 4, -math.pi / 4, -math.pi / 4]
    hb = [math.pi / 4, math.pi / 4, math.pi / 4, math.pi / 4 , math.pi / 4, math.pi / 4, math.pi / 4, math.pi / 4]
    numVars = 8
    simulatorType = "LeftRightSymmetricKneeQuad"
    string = "/Results/LeftRightSymmetricKneeQuad/"
    def __init__(self, inputVector):
        super(LeftRightSymmetricKneeQuadSimulator, self).__init__(0.0001, './data/skel/knee_quadriped.skel')
        self.controller = creature_controllers.LeftRightSymmetricKneeQuadController(self.skeletons[1], inputVector)
        self.skeletons[1].set_controller(self.controller)
    def getX0(self):
        x0 = np.random.uniform(low=-math.pi / 4, high=math.pi / 4, size=(self.numVars,))
        # x0 = np.append(x0Short, (np.random.uniform(low = 0, high=2*math.pi, size = 1)))
        return x0

class LRSymmetricPhaseKneeQuadSimulator(pydart.World):
    lb = [-math.pi / 4, -math.pi / 4, -math.pi / 4, -math.pi/ 4, -math.pi / 4, -math.pi / 4, -math.pi / 4, -math.pi / 4, 0]
    hb = [math.pi / 4, math.pi / 4, math.pi / 4, math.pi / 4 , math.pi / 4, math.pi / 4, math.pi / 4, math.pi / 4, 2*math.pi]
    numVars = 9
    simulatorType = "LRSymmetricPhaseKneeQuad"
    string = "/Results/LRSymmetricPhaseKneeQuad/"
    def __init__(self, inputVector):
        super(LRSymmetricPhaseKneeQuadSimulator, self).__init__(0.0001, './data/skel/knee_quadriped.skel')
        self.controller = creature_controllers.LRSymmetricPhaseKneeQuadController(self.skeletons[1], inputVector)
        self.skeletons[1].set_controller(self.controller)
    def getX0(self):
        x0Short = np.random.uniform(low=-math.pi / 4, high=math.pi / 4, size=(self.numVars - 1,))
        x0 = np.append(x0Short, (np.random.uniform(low = 0, high=2*math.pi, size = 1)))
        return x0



