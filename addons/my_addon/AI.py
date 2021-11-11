import numpy as np
import math

class AI:
    def __init__(self, speed):
        self.maxSpeed = float(100)
        # m/s
        self.speed = float(speed)
        # rad -pi à pi
        self.angle = 0

        self.prevCaptLine = np.array([0, 0, 1, 0, 0])
        #le buffer existe dans le cas si le robot ne commence pas totalement au centre de la ligne
        #pour qu'il puisse prendre connaissance de la ligne au début
        self.buffer = 4

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

    #Trouve le point moyen de la ligne sur le sol
    def averageLine(self, captLine):
        ##denum = (captLine[0] + 2 * captLine[1]+ 3 * captLine[2]+ 4 * captLine[3]+ 5 * captLine[4])/denum

        #s'assure qu'il n'y a pas de division par 0
        #if denum == 0:
        #    denum = 1

        return (captLine[0] + 2 * captLine[1]+ 3 * captLine[2]+ 4 * captLine[3]+ 5 * captLine[4])/5

    #fonction principal pour suivre la ligne
    def lineFollower(self, captLine, speed):
        #va chercher la valeur moyenne de l'array de la ligne actuel
        currLine = self.averageLine(captLine)
        #va chercher la valeur moyenne de l'array de la ligne précedente
        prevLine = self.averageLine(self.prevCaptLine)
        #trouve la difference entre la ligne précedente et la ligne suivante.
        #i.e. la pente de la tangente
        deltaLine = (currLine - prevLine)

        self.prevCaptLine = captLine

        #si la ligne est continue alors on garde le même capt
        if deltaLine != 0 :
            #fait un régle de trois pour normaliser la vitesse selon la vitesse selectionné
            speedModifier = speed / 100 * deltaLine

            if self.buffer > 0 :
                self.speed += round(speedModifier, 2)
            if self.speed > self.maxSpeed:
                self.speed = self.maxSpeed

            #calcule l'angle de correction a ajouter
            angleModifier = np.arctan([1 / deltaLine])

            #transforme la valeur de correction négative si on va à gauche
            if deltaLine < 0:
                angleModifier = -np.abs(angleModifier[0])
            else :
                angleModifier = angleModifier[0]

            if self.buffer > 0 :
                #self.angle += angleModifier
                #modulo considérant les nombres négative
                #arrondi l'angle pour réduire un possible surcorrection de l'angle
                self.angle = round(np.pi * math.trunc(angleModifier / (np.pi)), 2)
            else:
                self.buffer -= 1
        else:
            self.speed = speed
            #peut être cette ligne (85) n'est pas nécessaire
            self.angle = 0