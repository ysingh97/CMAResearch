import Creature_Simulators
import pydart2 as pydart
from cma import *
import numpy as np
from Creature_Simulators import *
import datetime
import multiprocessing as mp
import time
import math
import os
import sys

SIMULATOR = Creature_Simulators.LeftRightSymmetricWideFootQuadSimulator
general_option = {'maxiter': 20, 'popsize': 16}
DURATION = 7

OPTIONS = general_option
CMA_STEP_SIZE = 0.6
NUM_RESTART = 1
NUM_VARS = 3
BEST_FITNESS = math.inf
TIME = str(time.time())[1:str(time.time()).find(".")]
PATH = SIMULATOR.string + TIME
CPU_COUNT = mp.cpu_count()
GEN_COUNT = 0

def fitnessFunction(inputVector):
    simulator = SIMULATOR(inputVector)
    initialPos = simulator.skeletons[1].q
    endPos = episode(simulator)
    # result = 100000
    if endPos is not None:
        if endPos[4] < -.25:
            return 100000, inputVector
        result = endPos[3] - initialPos[3]
        zDiff = abs(endPos[5] - initialPos[5])
        fitness = result + 2 * zDiff
        # print("Zdiff: ", zDiff, "XDiff: ", result, "EndPos: ", endPos, "Initial Pos: ", initialPos)
        return fitness, inputVector
    else:
        return 100000, inputVector


def writeBest(fitness, vector):
    global GEN_COUNT
    s = "Generation: " + str(GEN_COUNT) + "; "
    s += "Duration: " + str(DURATION) + "; "
    s += "MaxIterations: " + str(general_option['maxiter']) + "; "
    s += "Population: " + str(general_option['popsize']) + "; "
    s += "Fitness: " + str(fitness) + "; "
    s += "Solution So Far: " + getSolutionString(vector) + ";\n\n"
    file = open("." + PATH + "/" + SIMULATOR.simulatorType + "_" + TIME + "_generationBest.txt", "a+")
    file.write(s)

FITNESS_FUNC =  fitnessFunction



def episode(current_simulator):

    terminal_flag = False
    while current_simulator.t < DURATION and not terminal_flag:
        # print(current_simulator.t)
        current_simulator.step()
        curr_q = current_simulator.skeletons[1].q
        if(curr_q[4] < -.25):
            break
        # print("XPos: ", curr_q[3])
        if (abs(curr_q) > 10 ** 3).any():
            # print(curr_q)
            # current_simulator.skeletons[1].controller.compute()
            # current_simulator.skeletons[1].controller.pd_controller_target_compute()
            terminal_flag = True
            print("NAN")
    res = current_simulator.skeletons[1].q
    if terminal_flag:
        res = None
    return res

def findBest(results):
    min = math.inf
    best = None
    for result in results:
        fitnessVectorTuple = results[result]
        if fitnessVectorTuple[0] < min:
            min = fitnessVectorTuple[0]
            best = fitnessVectorTuple
    return best


def run_CMA(x0):
    global OPTIONS, FITNESS_FUNC, CPU_COUNT, GEN_COUNT
    es = CMAEvolutionStrategy(x0, CMA_STEP_SIZE, OPTIONS)
    pool = mp.Pool(CPU_COUNT)
    while not es.stop():
        currentPopulation = es.ask()
        print("Current Population: ")
        print(currentPopulation)
        print("\n")
        results = {}
        for i in range(int(es.popsize/CPU_COUNT)):
            currentBatch = currentPopulation[(i * CPU_COUNT):(i + 1)*CPU_COUNT]
            batchResults = pool.map(FITNESS_FUNC, currentBatch)
            for result in batchResults:
                results[tuple(result[1].tolist())] = (result[0], result[1])
            # results.extend(batchResults)
        printGeneration(results)
        minGen = findBest(results)
        # print("minGen: ", minGen[0])
        writeBest(minGen[0], minGen[1])
        GEN_COUNT = GEN_COUNT + 1
        es.tell([x[1] for x in results.values()], [x[0] for x in results.values()])
    # es.optimize(FITNESS_FUNC)3
    print(results)
    res = es.result
    print(res)
    return res

def printDictionary(dictionary):
    s = ""
    for key in dictionary:
        s += "\t" + getSolutionString(key) + ": "
        s += "Fitness: " + str(dictionary[key][0]) + "\n"
    return s

def printGeneration(resultDict):
    global GEN_COUNT
    s = "Generation: " + str(GEN_COUNT)+ "; "
    s += "Duration: " + str(DURATION) + "; "
    s += "MaxIterations: " + str(general_option['maxiter']) + "; "
    s += "Population: " + str(general_option['popsize']) + ";\n "
    s += printDictionary(resultDict) + "\n\n"
    file = open("." + PATH + "/" + SIMULATOR.simulatorType + "_" + TIME + "_generations.txt", "a+")
    file.write(s)


def getSolutionString(result):
    stri = "Solution: ["
    length = len(result)
    # print(length)
    for i in range(length - 1):
        stri += str(result[i]) + ", "
    stri += str(result[length - 1]) + "]"
    return stri

def writeToFile(res, start, end):
    global FILENAME
    dateString = str(time.time())
    # print(dateString[1:dateString.find(".")])
    # print(SIMULATOR.string)
    s = "Duration: " + str(DURATION) + "; "
    s += "MaxIterations: " + str(general_option['maxiter']) + "; "
    s += "Population: " + str(general_option['popsize']) + "; "
    s += "Iterations: " + str(res.iterations) + "; "
    s += "CMA Time: " + str(end - start) + "; "
    s += getSolutionString(res.xbest) + "\n"
    # s += "Solution: " + str(res.xbest) + ";\n"
    file = open("." + PATH + "/" + SIMULATOR.simulatorType + "_" + TIME + "_best.txt", "a+")
    file.write(s)


if __name__=='__main__':
    pydart.init()
    # print(len(sys.argv) > 2)
    OPTIONS['bounds'] = SIMULATOR.lb, SIMULATOR.hb
    path = os.getcwd() + PATH
    # print(os.getcwd())
    # print(PATH)
    os.mkdir(path)
    # # # #
    x0 = SIMULATOR.getX0(SIMULATOR)
    # # x0 = np.random.uniform(low=-math.pi / 4, high=math.pi / 4, size=(SIMULATOR.numVars,))
    print(x0)
    # # # #
    start = time.time()
    res = run_CMA(x0)
    end = time.time()
    # print(end - start)
    # print(res.xbest)
    writeToFile(res, start, end)
    #
    # # #
    testSimulator = SIMULATOR(res.xbest)
    #
    # testSimulator = SIMULATOR([-0.756631626452491, -0.6976371074891565, 0.326702368122124, 0.023886411254193662, 0.5538665019740683, -0.034577243195564496, -0.49390035708827223, 0.42601978553710296, -0.7577052062056908, 0.3979802257162822, 0.6148985530895368, 0.3092823456786025])
    # fitnessFunction([-0.514072698768841, 0.7563476359519834, 0.7082520951648874, 0.2749209360574305, -0.7602852837370209, -0.6225817927147496, 0.6219895615201269, 0.4865330914798499])
    pydart.gui.viewer.launch_pyqt5(testSimulator)








