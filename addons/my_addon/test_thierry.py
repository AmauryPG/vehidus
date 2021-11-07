import bpy
from bpy import context as C
from bpy import data as D
import sys
import numpy as np
import addon_utils

# Parametres pour le render
FPS = 24
AI_UPDATE_RATE = 5
TIME_BETWEEN_AI_UPDATE = AI_UPDATE_RATE/FPS

# Parametres de distances
DISTANCE_X_CAPTEUR_LIGNE = 1.357
DISTANCE_Y_CAPTEUR_LIGNE = 0.18
DISTANCE_X_CAPTEUR_DISTANCE = 14.82

class ObjectCar :

    def __init__(self, ob, lineSensorsArray):
        self.ob = ob
        self.lineSensorsArray = lineSensorsArray
        self.facingAngle =  ob.rotation_euler[2]
        self.positionXYZ = [ob.location[0],ob.location[1],ob.location[2]]
        self.speed = 0
        
    def updateOb(self, frame):
        self.ob.location = [self.positionXYZ[0], self.positionXYZ[1], self.positionXYZ[2]]
        self.ob.rotation_euler[2] = self.facingAngle 
        self.updateSensorPos(frame)
        
    def updateSensorPos(self, frame):
        # Sensor Ligne
        self.lineSensorsArray[2].location = [(self.ob.location[0]+DISTANCE_X_CAPTEUR_LIGNE*np.cos(self.ob.rotation_euler[2])),
                                self.ob.location[1]+DISTANCE_Y_CAPTEUR_LIGNE*np.sin(self.ob.rotation_euler[2]),
                                0]
        self.lineSensorsArray[2].keyframe_insert(data_path="location", frame=frame)
        
        self.lineSensorsArray[0].location = [self.lineSensorsArray[2].location[0] - 2 * DISTANCE_X_CAPTEUR_LIGNE * np.sin(self.ob.rotation_euler[2]),
                              self.lineSensorsArray[2].location[1] + 2 * DISTANCE_Y_CAPTEUR_LIGNE * np.cos(self.ob.rotation_euler[2]),
                              0]
        self.lineSensorsArray[0].keyframe_insert(data_path="location", frame=frame)                    
        self.lineSensorsArray[1].location = [self.lineSensorsArray[2].location[0] - DISTANCE_X_CAPTEUR_LIGNE * np.sin(self.ob.rotation_euler[2]),
                              self.lineSensorsArray[2].location[1] + DISTANCE_Y_CAPTEUR_LIGNE * np.cos(self.ob.rotation_euler[2]),
                              0]
        self.lineSensorsArray[1].keyframe_insert(data_path="location", frame=frame)                      
        self.lineSensorsArray[3].location = [self.lineSensorsArray[2].location[0] + DISTANCE_X_CAPTEUR_LIGNE * np.sin(self.ob.rotation_euler[2]),
                              self.lineSensorsArray[2].location[1] - DISTANCE_Y_CAPTEUR_LIGNE * np.cos(self.ob.rotation_euler[2]),
                              0]
        self.lineSensorsArray[3].keyframe_insert(data_path="location", frame=frame)                    
        self.lineSensorsArray[4].location = [self.lineSensorsArray[2].location[0] + 2 * DISTANCE_X_CAPTEUR_LIGNE * np.sin(self.ob.rotation_euler[2]),
                              self.lineSensorsArray[2].location[1] - 2 * DISTANCE_Y_CAPTEUR_LIGNE * np.cos(self.ob.rotation_euler[2]),
                              0]
        self.lineSensorsArray[4].keyframe_insert(data_path="location", frame=frame)
        #Sensor distance
        self.distSensor = [self.ob.location[0]+DISTANCE_X_CAPTEUR_DISTANCE*np.cos(self.ob.rotation_euler[2]),
                        self.ob.location[0]+DISTANCE_X_CAPTEUR_DISTANCE*np.sin(self.ob.rotation_euler[2])]
        
        

def turnAngleSpeed(angle, speed, lastFrame, car):
    facingAngle = car.facingAngle
    turn = [facingAngle, facingAngle + angle]
    frames = [lastFrame,  lastFrame + AI_UPDATE_RATE]
    distance = TIME_BETWEEN_AI_UPDATE * speed
    positionFinalX = [car.positionXYZ[0], car.positionXYZ[0] + np.cos(facingAngle + angle)*distance]
    positionFinalY = [car.positionXYZ[1],car.positionXYZ[1] + np.sin(facingAngle + angle)*distance]
    
    for i in range(len(frames)):
        frame = frames[i]
        bpy.context.scene.frame_set(frame)
        car.positionXYZ[0] = positionFinalX[i]
        car.positionXYZ[1] = positionFinalY[i]
        car.facingAngle = turn[i]
        car.updateOb(frame)
        car.ob.keyframe_insert(data_path="location", frame=frame)
        car.ob.keyframe_insert(data_path="rotation_euler", frame=frame)
    return lastFrame + AI_UPDATE_RATE

    
    
    
if __name__ == "__main__":
    lastFrame = 1
    ob = bpy.data.objects["Voiture"]
    lineSensorsArray = [None] * 5
    for i in range(5):
        lineSensorsArray[i] = bpy.data.objects["SensorLine" + str(i)]
    car = ObjectCar(ob,lineSensorsArray)
    car.positionXYZ = [0,0,0.385]
    car.facingAngle = 0
    car.updateOb(lastFrame)
    for i in range(0,10):
        lastFrame = turnAngleSpeed(0,10,lastFrame,car)
    for i in range(0,10):
        lastFrame = turnAngleSpeed(np.pi/5,10,lastFrame,car)
        
    print("position de la voiture:" + str(car.positionXYZ))
