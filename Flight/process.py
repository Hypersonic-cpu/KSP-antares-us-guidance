from math import *

def ThrustTimeCurve():
    timeFile = open("D:/Program/Project/KSP_Control_copy - 副本/Flight/timeX.txt", "r")
    timeLis = list(timeFile)
    timeFile.close()

    thrustFile = open("D:/Program/Project/KSP_Control_copy - 副本/Flight/thrustX.txt", "r")
    thrustLis = list(thrustFile)
    thrustFile.close()

    massFile = open("D:/Program/Project/KSP_Control_copy - 副本/Flight/massX.txt", "r")
    massLis = list(massFile)
    massFile.close()

    outFile = open("D:/Program/Project/KSP_Control_copy/Flight/outX.txt", "w")

    # print(floor(float(timeLis[0]) * 10))
    T0 = float(timeLis[0])
    for i in range(len(timeLis)):
        outFile.write('{ ' + str(floor(10 * (float(timeLis[i]) - T0)))
                    + ', Tuple.Create( ' + str(float(thrustLis[i])) + ', ' + str(float(massLis[i])) + ' ) },\n')

def MassTimeCurve():
    timeFile = open("D:/Program/Project/KSP_Control_copy - 副本/Flight/timeX.txt", "r")
    timeLis = list(timeFile)
    timeFile.close()

    thrustFile = open("D:/Program/Project/KSP_Control_copy - 副本/Flight/massX.txt", "r")
    thrustLis = list(thrustFile)
    thrustFile.close()

    outFile = open("D:/Program/Project/KSP_Control_copy - 副本/Flight/massOutX.txt", "w")

    T0 = float(timeLis[0])
    for i in range(len(timeLis)):
        outFile.write('{ ' + str(floor(10 * (float(timeLis[i]) - T0)))
                    + ', ' + str(float(thrustLis[i])) + ' },\n')

ThrustTimeCurve()