import numpy as np

class AI:
    def __init__(self):
        self.maxSpeed = float(1)
        # m/s
        self.speed = float(0)
        # rad -pi à pi
        self.angle = 0

        self.prevCaptLine = np.array([0, 0, 0, 0, 0])

    def updateAI(self, captLine, captUltra):
        self.averageLine(captLine)

    def averageLine(self, captLine):
        denum = captLine[0] + captLine[1]+ captLine[2]+ captLine[3]+ captLine[4]

        if denum == 0:
            denum = 1

        return (captLine[0] + 2 * captLine[1]+ 3 * captLine[2]+ 4 * captLine[3]+ 5 * captLine[4])/denum

    def lineFollower(self, captLine, speed):
        currLine = self.averageLine(captLine)
        prevLine = self.averageLine(self.prevCaptLine)
        deltaLine = (currLine - prevLine)

        if currLine != prevLine :
            speedModifier = speed / 100 * deltaLine

            self.prevCaptLine = captLine

            self.speed += round(speedModifier, 2)
            print("speed ", self.speed)

            angleModifier = np.tan([1 / deltaLine])

            if deltaLine < 0:
                angleModifier = -np.abs(angleModifier[0])
            else :
                angleModifier = np.abs(angleModifier[0])

            self.angle += round(angleModifier, 2)
            self.angle = self.angle % np.pi
            print("angle ", self.angle)