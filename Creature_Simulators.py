import pydart2 as pydart
import creature_controllers
import math
import time
import numpy as np
import random
from pydart2.contact import Contact

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
    lb = [-math.pi / 4 -math.pi, -math.pi / 4, -math.pi, -math.pi / 4, -math.pi, -math.pi / 4, -math.pi] * 2
    hb = [math.pi / 4, math.pi, math.pi / 4, math.pi, math.pi / 4, math.pi, math.pi / 4, math.pi] * 2
    numVars = 16
    string = "/Results/KneeQuad/"
    simulatorType = "KneeQuad"
    def getX0(self):
        x0 = np.random.uniform(low=-math.pi / 4, high=math.pi / 4, size=(self.numVars,))
        # x0 = np.append(x0Short, (np.random.uniform(low = 0, high=2*math.pi, size = 1)))
        return x0
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

class LeftRightSymmetricCapsuleQuadSimulator(pydart.World):
    lb = [-math.pi / 4, -math.pi / 4, -math.pi / 4, -math.pi/ 4, -math.pi / 4, -math.pi / 4, -math.pi / 4, -math.pi / 4]
    hb = [math.pi / 4, math.pi / 4, math.pi / 4, math.pi / 4 , math.pi / 4, math.pi / 4, math.pi / 4, math.pi / 4]
    numVars = 8
    simulatorType = "LeftRightSymmetricCapsuleQuad"
    string = "/Results/LeftRightSymmetricCapsuleQuad/"
    def __init__(self, inputVector):
        super(LeftRightSymmetricCapsuleQuadSimulator, self).__init__(0.0001, './data/skel/knee_quadriped_capsule_leg.skel')
        try:
            self.set_collision_detector(3)
        except Exception as e:
            print('Does not have ODE collision detector, reverted to bullet collision detector')
            self.set_collision_detector(2)
        self.controller = creature_controllers.LeftRightSymmetricCapsuleQuadController(self.skeletons[1], inputVector)
        self.skeletons[1].set_controller(self.controller)
    def getX0(self):
        x0 = np.random.uniform(low=-math.pi / 4, high=math.pi / 4, size=(self.numVars,))
        # x0 = np.append(x0Short, (np.random.uniform(low = 0, high=2*math.pi, size = 1)))
        return x0

class LeftRightSymmetricWideFootQuadSimulator(pydart.World):
    lb = [-math.pi / 4] * 12
    hb = [math.pi / 4] * 12
    numVars = 12
    simulatorType = "LeftRightSymmetricWideFootQuad"
    string = "/Results/LeftRightSymmetricWideFootQuad/"
    def __init__(self, inputVector):
        # super(LeftRightSymmetricWideFootQuadSimulator, self).__init__(0.0001, './data/skel/wide_foot_quadriped_uniform_mass.skel')
        super(LeftRightSymmetricWideFootQuadSimulator, self).__init__(0.0001, './data/skel/wide_foot_quadriped.skel')

        self.controller = creature_controllers.LeftRightSymmetricWideFootQuadController(self.skeletons[1], inputVector)
        self.skeletons[1].set_controller(self.controller)
        # print(self.skeletons[1].M.shape)
    def getX0(self):
        # x0 = []
        # for i in range(8):
        #     x0.append(random.uniform(-math.pi/4, math.pi/4))
        # x0.append(random.uniform(-.3, .3))
        # x0.append(random.uniform(-math.pi / 4, math.pi / 4))
        # x0.append(random.uniform(-.3, .3))
        # x0.append(random.uniform(-math.pi / 4, math.pi / 4))
        x0 = np.random.uniform(low=-math.pi / 4, high=math.pi / 4, size=12)
        # x0Second = np.append(x0First, np.random.uniform(low=-.3, high=.3, size=1))
        # x0Third = np.append(x0Second, np.random.uniform(low=-math.pi / 4, high=math.pi / 4, size=1))
        # x0Fourth = np.append(x0Third, (np.random.uniform(low=-.3, high=.3, size=1)))
        # x0 = np.append(x0Fourth, np.random.uniform(low=-math.pi / 4, high=math.pi / 4, size=1))
        return x0


class SymmetricWideFootQuadSimulator(pydart.World):
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

class SpiderSimulator(pydart.World):
    lb = [(-3 *math.pi) / 8] * 32
    hb = [(3 *math.pi) / 8] * 32
    numVars = 32
    simulatorType = "Spider"
    string = "/Results/Spider/"
    def __init__(self, inputVector):
        super(SpiderSimulator, self).__init__(0.0001, './data/skel/spider.skel')
        try:
            self.set_collision_detector(2)
        except Exception as e:
            print('Does not have ODE collision detector, reverted to bullet collision detector')
            self.set_collision_detector(2)
        self.controller = creature_controllers.SpiderController(self.skeletons[1], inputVector)
        self.skeletons[1].set_controller(self.controller)
    def getX0(self):
        x0 = np.random.uniform(low=(-3 *math.pi) / 8, high=(3 *math.pi) / 8, size=(self.numVars,))
        return x0
    # def render_with_ri(self, ri):
    #     p0, p1 = self.controller.getNormalEndpoints()
    #     print(p0, p1)
    #     if p0 is not None and p1 is not None:
    #         ri.set_color(1.0, 0.0, 0.0)
    #         ri.render_arrow(p0, p1, r_base=0.05, head_width=0.1, head_len=0.1)
        # if self.force is not None and self.duration >= 0:
        #     p0 = self.skeletons[1].body('h_spine').C
        #     p1 = p0 + 0.01 * self.force

    def render(self,
               render_markers=True,
               render_contacts=False,
               render_contact_size=0.01,
               render_contact_force_scale=-0.005):
        super(SpiderSimulator, self).render(render_markers, render_contacts, render_contact_size, render_contact_force_scale)

        p0, p1 = self.controller.getNormalEndpoints()
        test_vec = np.array([0, 100, 0])
        fake_contact = Contact(self, np.concatenate([p0, 100*(p1-p0), [0, 0, 0, 0]]))

        fake_contact.render(size=render_contact_size,
                            scale=-render_contact_force_scale)


class BasicSpiderSimulator(pydart.World):
    numVars = 16
    lb = [(-3 *math.pi) / 8] * numVars
    hb = [(3 *math.pi) / 8] * numVars

    simulatorType = "BasicSpider"
    string = "/Results/BasicSpider/"
    def __init__(self, inputVector):
        super(BasicSpiderSimulator, self).__init__(0.0001, './data/skel/spider.skel')
        try:
            self.set_collision_detector(2)
        except Exception as e:
            print('Does not have ODE collision detector, reverted to bullet collision detector')
            self.set_collision_detector(2)
        self.controller = creature_controllers.BasicSpiderController(self.skeletons[1], inputVector)
        self.skeletons[1].set_controller(self.controller)
    def getX0(self):
        x0 = np.random.uniform(low=(-3 *math.pi) / 8, high=(3 *math.pi) / 8, size=(self.numVars,))
        return x0



