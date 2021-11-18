import numpy as np

distance_entre_capteurs = 0.18
distance_entre_centre_masse_et_capteur = 1.41
class AI_Thierry :
    def __init__(self, speed, car):
        self.maxSpeed = float(100)
        # m/s
        self.speed = float(speed)
        # rad -pi à pi
        self.angle = 0

        self.prevCaptLine = np.array([0, 0, 1, 0, 0])
    def updateAi(donnees_ligne) :
        match donnees_ligne :
            case [1,0,0,0,0] : 
                self.angle = np.arctan(2 * distance_entre_capteurs/distance_entre_centre_masse_et_capteur)
            case [1,1,0,0,0] :
                self.angle = np.arctan(1.5 * distance_entre_capteurs/distance_entre_centre_masse_et_capteur)
            case [0,1,0,0,0] :
                self.angle = np.arctan(1 * distance_entre_capteurs/distance_entre_centre_masse_et_capteur)
            case [0,1,1,0,0] :
                self.angle = np.arctan(0.5 * distance_entre_capteurs/distance_entre_centre_masse_et_capteur)
            case [0,0,1,0,0] :
                self.angle = self.angle
            case [0,0,1,1,0] :
                self.angle = -np.arctan(0.5 * distance_entre_capteurs/distance_entre_centre_masse_et_capteur)
            case [0,0,0,1,0] :
                self.angle = -np.arctan(1.0 * distance_entre_capteurs/distance_entre_centre_masse_et_capteur)
            case [0,0,0,1,1] :
                self.angle = -np.arctan(1.5 * distance_entre_capteurs/distance_entre_centre_masse_et_capteur)
            case [0,0,0,0,1] :
                self.angle = -np.arctan(2 * distance_entre_capteurs/distance_entre_centre_masse_et_capteur)
            case _ :
                self.angle = self.angle
        return self.angle            