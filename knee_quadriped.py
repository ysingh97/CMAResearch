# Copyright (c) 2015, Disney Research
# All rights reserved.
#
# Author(s): Sehoon Ha <sehoon.ha@disneyresearch.com>
# Disney Research Robotics Group
import pydart2 as pydart
import numpy as np
import math
import cma
import os



class Controller(object):
    def __init__(self, skel):
        self.skel = skel
        self.target = None
        self.Kp = np.array([0.0] * 6 + [400.0] * (self.skel.ndofs - 6))
        self.Kd = np.array([0.0] * 6 + [40.0] * (self.skel.ndofs - 6))
        self.vector = [1, math.pi/2, 1, math.pi/2, 1, math.pi/2, 1, math.pi/2]
        self.vector = [math.pi/12, 0, math.pi/4, 0, 0, 0, 0, 0] * 4

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
        # print("Position: ", self.skel.q)
        # print("Xpos: ", self.skel.q['joint_2_pos_x'])
        return -self.Kp * (self.skel.q - self.target) - self.Kd * self.skel.dq


if __name__ == '__main__':
    print('Hello, PyDART!')

    pydart.init()
    print('pydart initialization OK')

    world = pydart.World(0.0002, './data/skel/spider.skel')
    try:
        world.set_collision_detector(2)
    except Exception as e:
        print('Does not have ODE collision detector, reverted to bullet collision detector')
        world.set_collision_detector(2)
    print('pydart create_world OK')

    spiderSkel = world.skeletons[1]
    # print(quadripedSkel.q)
    # bodynodes = inchwormSkel.bodynodes
    # for node in bodynodes:
    #     print("friction: ", node.friction_coeff())
    controller = Controller(spiderSkel)
    spiderSkel.set_controller(controller)
    pydart.gui.viewer.launch(world)
    # path = os.getcwd()
    # os.mkdir(path + "/test")

