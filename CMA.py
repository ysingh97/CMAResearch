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

SIMULATOR = Creature_Simulators.SpiderSimulator
general_option = {'maxiter': 25, 'popsize': 32}
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
        # if endPos[4] < -.25:
        #     print("")
        #     return 100000, inputVector
        result = endPos[3] - initialPos[3]
        zDiff = abs(endPos[5] - initialPos[5])
        fitness = result + 2 * zDiff
        normalDot = simulator.controller.getNormalDot()
        fitness = fitness + (1-normalDot)*1000
        # print("Zdiff: ", zDiff, "XDiff: ", result, "EndPos: ", endPos, "Initial Pos: ", initialPos)
        return fitness, inputVector
    else:
        print("simulation exploded")
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
        if (abs(curr_q) > 10 ** 3).any() or current_simulator.controller.getNormalDot() <= .9:
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
    iterations = 0
    while not es.stop():
        print("Iteration: ", iterations)
        iterations = iterations + 1
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


    #20 iterations, pop 16 , LRWideFoot
    # testSimulator = SIMULATOR([-0.10174790550070655, -0.15749526831654012, -0.24799156360211141, -0.08646927860179397, 0.6154454121591943, -0.076681768580598, -0.5994726514424225, 0.025171940385812892, 0.4185361695932176, 0.49278145857619593, -0.7850888202256414, -0.3566908552379263])
    #30 iteraitons, pop 32, LRWideFoot
    # testSimulator = SIMULATOR([-0.10174790550070655, -0.15749526831654012, -0.24799156360211141, -0.08646927860179397, 0.6154454121591943, -0.076681768580598, -0.5994726514424225, 0.025171940385812892, 0.4185361695932176, 0.49278145857619593, -0.7850888202256414, -0.3566908552379263])
    # 30 iteraitons, pop 32, LRWideFoot
    # testSimulator = SIMULATOR([-0.40495834939720693, 0.1297391453749488, 0.21685380195482762, 0.4603129572566932, 0.3622253402312553, 0.5121369486738435, 0.4736273762282262, -0.01180051501288637, 0.16516001634441047, -0.049808936507034135, 0.4058344535132633, 0.5939201894261522])

    # 30 iterations, 12 pop, LRSymmetricPhaseKnee
    # testSimulator = SIMULATOR([0.1270850793200799, 0.5047804513933636, -0.15166891244470648, 0.2505405941449012, 0.7790518772406382, 0.36207638518478436, -0.02597488981307873, -0.4483084363352511, 0.06535545253940811])

    #30 iterations, 32 pop, LeftRightSymmetricQuad
    # testSimulator = SIMULATOR([-0.01573170417502552, -0.3552679299176819, 0.15191754355913126, 0.09216630127699157, 0.7702752758916498, 0.27074152227014553, -0.04790882933158183, -0.2537345655360871])

    #30 iterations, 32 pop, LRSymWideFoot
    # testSimulator = SIMULATOR([0.383078139874705, -0.7598245044184002, -0.5556875776002632, 0.044583837682674154, 0.1563882572427746, 0.3536917035604654, 0.6383543198531132, -0.38257913500165447, 0.00985264711215017, 0.24383230306256842, 0.16679394197442338, -0.08933471957914452])

    #40 iterations, 48 pop, LRWideFoot
    # testSimulator = SIMULATOR([0.46630226866904523, -0.09035613239452997, 0.41008355593203305, -0.6614990165896063, -0.6172943395187678,
    #  -0.7629010297653951, 0.5073885668913174, -0.21858281247631792, -0.2586085899441053, -0.5940746689855843,
    #  0.782851567370281, -0.3060416128977228])

    #spider all dof
    # testSimulator = SIMULATOR([0.3723501318863691, 1.1678595960839597, -0.1569258221476587, 0.24495637346806864, 1.0778045197202937, -0.9855109443998904, 1.0583464289299256, -0.08774364762497212, -1.1149586949514558, -1.134175398994463, 0.9257096084594617, -0.372952021676588, -1.0372699989256868, -0.8387960147903675, 0.5883142027970178, 0.6971646208739037, 0.5136742069811631, -0.5505050908529391, 1.0925744326681441, -0.026458244859809965, 1.0870814597191574, 0.0931135915468222, -0.37946413309817023, 0.21809491862819586, 0.6342053472769811, -0.7728460968456383, 1.0101824139530957, -0.9448652418916719, 0.31985896023026333, -0.5434757655339815, 0.6238802853909565, -0.9290562998005504])
    
    #just first joints
    # testSimulator = SIMULATOR([0.7303540117359049, 1.029788001017416, 1.0038273549565444, 0.9221910710583081, 1.1456469751809306, 0.3914514738142889, 1.0071479857942354, 1.084099216318121, -0.9272175820503881, -0.799735512437346, 1.1707966213883096, 0.8732789726661583, -0.22481711454391462, -1.1151194436722058, 0.4227663555348966, 0.7442111131633926])
    pydart.gui.viewer.launch_pyqt5(testSimulator)







