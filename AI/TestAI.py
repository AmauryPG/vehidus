from AI import *
import numpy as np

speed = 100

def inRange(value, refer, tolerance):
    return refer + tolerance >= value >= refer - tolerance

def individualTestLineFollower(line, expectedSpeed, speedTolerance, expectedAngle, angleTolerance):
    ai = AI()

    sucessTestSpeed = 0
    sucessTestAngle = 0

    for i in range(len(line)):
        ai.lineFollower(line[i], speed)
        if inRange(ai.getSpeed(), expectedSpeed[i], speedTolerance):
            sucessTestSpeed += 1
        if inRange(ai.getAngle(), expectedAngle[i], angleTolerance):
            sucessTestAngle += 1

    print("RESULTS")
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
    expectedSpeed = np.array([3.1,
                              3.1,
                              3.1,
                              2.1,
                              2.1,
                              3.1,
                              3.1,
                              3.1,
                              4.1,
                              4.1,
                              4.1,
                              4.6,
                              5.1,
                              5.1,
                              5.1,
                              5.1,
                              3.0999999999999996,
                              3.0999999999999996,
                              3.0999999999999996,
                              3.0999999999999996])

    angleTolerance = 0.1
    expectedAngle = np.array([0.35,
                              0.35,
                              0.35,
                              1.9315926535897932,
                              1.9315926535897932,
                              0.3500000000000001,
                              0.3500000000000001,
                              0.3500000000000001,
                              1.9100000000000001,
                              1.9100000000000001,
                              1.9100000000000001,
                              0.9584073464102065,
                              0.0068146928204133594,
                              0.0068146928204133594,
                              0.0068146928204133594,
                              0.0068146928204133594,
                              2.598407346410206,
                              2.598407346410206,
                              2.598407346410206,
                              2.598407346410206])

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