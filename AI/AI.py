import numpy as np
import math

#en cm
maxCaptUltra = 100
#en m/s
maxSpeed = 100
#en rad
maxAngle = np.pi

#état de la machine
LINE_FOLLOWER            = 0
DODGE_OBJECT_ANALYZE     = 1
DODGE_OBJECT_EDGE_UP     = 2
DODGE_OBJECT_EDGE_DOWN   = 3
DODGE_OBJECT_RETURN_LINE = 4

class AI:
    def __init__(self, speed):
        # m/s
        self.speed = float(speed)
        # rad -pi à pi
        self.angle = 0

        self.prevCaptLine = np.array([0, 0, 1, 0, 0])
        #le buffer existe dans le cas si le robot ne commence pas totalement au centre de la ligne
        #pour qu'il puisse prendre connaissance de la ligne au début
        self.buffer = 4
        #mesure en cm
        self.treshold = 10
        self.state = LINE_FOLLOWER

    #fonction principal de l'AI
    def updateAI(self, captLine, captUltra):
        self.averageLine(captLine)

    #getter et setter
    def getSpeed(self):
        return self.speed

    def setSpeed(self, speed):
        self.speed = speed

    def getAngle(self):
        return  self.angle

    def setAngle(self, angle):
        self.angle = angle

    def dodgeObject(self, captUltra):
        if self.state == DODGE_OBJECT_ANALYZE:
            if captUltra < maxCaptUltra:
                #tourne à droite de pi/20
                self.angle += 0.157
                self.speed = maxSpeed
            else:
                #changer d'état lorsqu'on est a la limite de l'objet a contourner
                self.state = DODGE_OBJECT_EDGE_DOWN
                self.angle = 0
        elif self.state == DODGE_OBJECT_EDGE_UP:
            self.angle = maxAngle
            self.speed = 0.3 * self.speed
        elif self.state == DODGE_OBJECT_EDGE_DOWN:
            if captUltra > maxCaptUltra:
                #la voiture a fini de contourner l'objet
                self.state = DODGE_OBJECT_RETURN_LINE
            else:
                self.angle = -maxAngle
                self.speed = 0.3 * self.speed
        elif self.state == DODGE_OBJECT_RETURN_LINE:
            self.angle = -maxAngle
            self.speed = 0.3 * self.speed


