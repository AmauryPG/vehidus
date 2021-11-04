from AI import *
import numpy as np

speed = 100

def inRange(value, refer, tolerance):
    return refer + tolerance >= value >= refer - tolerance

def individualTestLineFollower(line, expectedSpeed, speedTolerance, expectedAngle, angleTolerance):
    ai = AI(speed)

    sucessTestSpeed = 0
    sucessTestAngle = 0

    for i in range(len(line)):
        ai.lineFollower(line[i], speed)
        if inRange(ai.getSpeed(), expectedSpeed[i], speedTolerance):
            sucessTestSpeed += 1
        if inRange(ai.getAngle(), expectedAngle[i], angleTolerance):
            sucessTestAngle += 1

    print("RESULTS TEST SUCESS POURCENTANGE")
    print("SPEED : ", 100 * sucessTestSpeed / len(line), " %")
    print("ANGLE : ", 100 * sucessTestAngle / len(line), " %") 

def testLineFollower():
    print("BEGIN : Test #0 line follower")
    speedTolerance = 0.1
    expectedSpeed = np.array([speed,
                              speed,
                              speed,
                              speed,
                              speed,
                              speed,
                              speed,
                              speed,
                              speed,
                              speed,
                              speed,
                              speed,
                              speed,
                              speed,
                              speed,
                              speed,
                              speed,
                              speed,
                              speed,
                              speed])

    angleTolerance = 0.1
    expectedAngle = np.array([0,
                              0,
                              0,
                              0,
                              0,
                              0,
                              0,
                              0,
                              0,
                              0,
                              0,
                              0,
                              0,
                              0,
                              0,
                              0,
                              0,
                              0,
                              0,
                              0])

    line = np.array([[0, 0, 1, 0, 0],
                    [0, 0, 1, 0, 0],
                    [0, 0, 1, 0, 0],
                    [0, 0, 1, 0, 0],
                    [0, 0, 1, 0, 0],
                    [0, 0, 1, 0, 0],
                    [0, 0, 1, 0, 0],
                    [0, 0, 1, 0, 0],
                    [0, 0, 1, 0, 0],
                    [0, 0, 1, 0, 0],
                    [0, 0, 1, 0, 0],
                    [0, 0, 1, 0, 0],
                    [0, 0, 1, 0, 0],
                    [0, 0, 1, 0, 0],
                    [0, 0, 1, 0, 0],
                    [0, 0, 1, 0, 0],
                    [0, 0, 1, 0, 0],
                    [0, 0, 1, 0, 0],
                    [0, 0, 1, 0, 0],
                    [0, 0, 1, 0, 0]])

    individualTestLineFollower(line, expectedSpeed, speedTolerance, expectedAngle, angleTolerance)

    print("END : Test #0 line follower")
    print()

    print("BEGIN : Test #1 line follower")
    speedTolerance = 0.1
    expectedSpeed = np.array([
                            100,
                            100,
                            100,
                            99,
                            100,
                            100,
                            100,
                            100,
                            100,
                            100,
                            100,
                            100,
                            100,
                            100,
                            100,
                            100,
                            98,
                            100,
                            100,
                            100])

    angleTolerance = 0.1
    expectedAngle = np.array([
                            0,
                            0,
                            0,
                            - 1.56,
                            0,
                            1.56,
                            0,
                            0,
                            1.56,
                            0,
                            0,
                            - 2.19,
                            - 1.23,
                            0,
                            0,
                            0,
                            - 0.55,
                            0,
                            0,
                            0])

    line = np.array([[0, 0, 1, 0, 0],
                    [0, 0, 1, 0, 0],
                    [0, 0, 1, 0, 0],
                    [0, 1, 0, 0, 0],
                    [0, 1, 0, 0, 0],
                    [0, 0, 1, 0, 0],
                    [0, 0, 1, 0, 0],
                    [0, 0, 1, 0, 0],
                    [0, 0, 0, 1, 0],
                    [0, 0, 0, 1, 0],
                    [0, 0, 0, 1, 0],
                    [0, 0, 0, 1, 1],
                    [0, 0, 0, 0, 1],
                    [0, 0, 0, 0, 1],
                    [0, 0, 0, 0, 1],
                    [0, 0, 0, 0, 1],
                    [0, 0, 1, 0, 0],
                    [0, 0, 1, 0, 0],
                    [0, 0, 1, 0, 0],
                    [0, 0, 1, 0, 0]])

    individualTestLineFollower(line, expectedSpeed, speedTolerance, expectedAngle, angleTolerance)

    print("END : Test #1 line follower")