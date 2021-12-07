import numpy as np 
import math

class AI:
    def __init__(self, speed):
        self.maxSpeed = float(100)
        # m/s
        self.speed = float(speed)
        self.buffer = 0
        # rad -pi à pi
        self.angle = 0
        self.turnValue = 3
        self.prevCaptLine = 3
        #le buffer existe dans le cas si le robot ne commence pas totalement au centre de la ligne
        #pour qu'il puisse prendre connaissance de la ligne au début
        self.buffer = 4
        self.angleModifier = 0
    #fonction principal de l'AI
    def updateAI(self, captLine, captUltra):
        line = self.averageLine(captLine)
    #getter et setter
    def getSpeed(self):
        return self.speed
    def setSpeed(self, speed):
        self.speed = speed
    def getAngle(self):
        return  self.angle
    def setAngle(self, angle):
        self.angle = angle
    #Trouve le point moyen de la ligne sur le sol
    def averageLine(self, captLine):
        denum = captLine[0] + captLine[1]+ captLine[2]+ captLine[3]+ captLine[4]
        #s'assure qu'il n'y a pas de division par 0
        if denum == 0:
            #cas ou il n'y a pas de limite
            return 3
        return (captLine[0] + 2 * captLine[1]+ 3 * captLine[2]+ 4 * captLine[3]+ 5 * captLine[4])/denum
    #fonction principal pour suivre la ligne
    def lineFollower(self, captLine, speed): 
        #va chercher la valeur moyenne de l'array de la ligne actuel
        currLine = self.averageLine(captLine)
        #va chercher la valeur moyenne de l'array de la ligne précedente
        #trouve la difference entre la ligne précedente et la ligne suivante.
        #i.e. la pente de la tangente
        deltaLine = (currLine - 3)
        if deltaLine == 0:
            self.speed = speed
            self.angle = self.angle + self.angleModifier
            self.angleModifier = self.angleModifier * (1/20)
        else:                
            #0.0558 : 3.2
            #0.0175 : 1
            self.angleModifier = np.tan(np.deg2rad(1)/deltaLine)
            self.angle = self.angle*0.4 - self.angleModifier
            if currLine <= 2 or currLine >= 4:
                self.angle = 1.8 * self.angle
            if deltaLine < 0:
                self.angle = abs(self.angle)
            else:
                self.angle = -abs(self.angle)
            if abs(self.angle) > 0:
                g = abs(2 / (abs(self.angle) + 1) - 1)
            else :
                g = 1
            if abs(self.angle) >= np.pi:
                h = -1
            else:
                h = 1
            self.speed = self.speed * g * h
            self.prevCaptLine = currLine