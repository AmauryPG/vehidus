import bpy
from bpy import context as C
from bpy import data as D
import sys
import numpy as np
import addon_utils

FPS = 24
AI_UPDATE_RATE = 5
TIME_BETWEEN_AI_UPDATE = AI_UPDATE_RATE/FPS


class ObjectCar :

    def __init__(self, ob,Line0,Line1,Line2,Line3,Line4):
        self.ob=ob
        self.Line0 = Line0
        self.Line1 = Line1
        self.Line2 = Line2
        self.Line3 = Line3
        self.Line4 = Line4
        
        self.facingAngle =  ob.rotation_euler[2]
        self.positionXYZ = [ob.location[0],ob.location[1],ob.location[2]]
        self.lineSensor = []
        self.lineSensor.append([0,0])
        self.lineSensor.append([0,0])
        self.lineSensor.append([0,0])
        self.lineSensor.append([0,0])
        self.lineSensor.append([0,0])
        self.speed = 0
        
    def updateOb(self):
        self.ob.location[0] = self.positionXYZ[0] 
        self.ob.location[1] = self.positionXYZ[1] 
        self.ob.location[2] = self.positionXYZ[2]
        self.ob.rotation_euler[2] = self.facingAngle 
        self.updateSensorPos()
        
    def updateSensorPos(self):
        distanceX = 1.357
        distanceY = 0.18
        distanceXDist = 14.82
        # Sensor Ligne
        self.lineSensor[2] = [(self.ob.location[0]+distanceX*np.cos(self.ob.rotation_euler[2])),
                                self.ob.location[1]+distanceY*np.sin(self.ob.rotation_euler[2])]
        
        self.lineSensor[0] = [self.lineSensor[2][0] - 2 * distanceX * np.sin(self.ob.rotation_euler[2]),
                              self.lineSensor[2][1] + 2 * distanceY * np.cos(self.ob.rotation_euler[2])]
                            
        self.lineSensor[1] = [self.lineSensor[2][0] - distanceX * np.sin(self.ob.rotation_euler[2]),
                              self.lineSensor[2][1] + distanceY * np.cos(self.ob.rotation_euler[2])]
                              
        self.lineSensor[3] = [self.lineSensor[2][0] + distanceX * np.sin(self.ob.rotation_euler[2]),
                              self.lineSensor[2][1] - distanceY * np.cos(self.ob.rotation_euler[2])]
                            
        self.lineSensor[4] = [self.lineSensor[2][0] + 2 * distanceX * np.sin(self.ob.rotation_euler[2]),
                              self.lineSensor[2][1] - 2 * distanceY * np.cos(self.ob.rotation_euler[2])]
                              
        # positionnement des cubes sur les sensors
        self.Line0.location[0] = self.lineSensor[0][0]
        self.Line0.location[1] = self.lineSensor[0][1]
        
        self.Line1.location[0] = self.lineSensor[1][0]
        self.Line1.location[1] = self.lineSensor[1][1]
        
        self.Line2.location[0] = self.lineSensor[2][0]
        self.Line2.location[1] = self.lineSensor[2][1]
        
        self.Line3.location[0] = self.lineSensor[3][0]
        self.Line3.location[1] = self.lineSensor[3][1]
        
        self.Line4.location[0] = self.lineSensor[4][0]
        self.Line4.location[1] = self.lineSensor[4][1]
        
        #Sensor distance
        self.distSensor = [self.ob.location[0]+distanceXDist*np.cos(self.ob.rotation_euler[2]),self.ob.location[0]+distanceXDist*np.sin(self.ob.rotation_euler[2])]
        
        

def turnAngleSpeed(angle, speed, lastFrame, car):
    facingAngle = car.facingAngle
    turn = [facingAngle, facingAngle + angle]
    frames = [lastFrame,  lastFrame + AI_UPDATE_RATE]
    distance = TIME_BETWEEN_AI_UPDATE * speed
    print(distance)
    positionFinalX = [car.positionXYZ[0], car.positionXYZ[0] + np.cos(facingAngle + angle)*distance]
    positionFinalY = [car.positionXYZ[1],car.positionXYZ[1] + np.sin(facingAngle + angle)*distance]
    
    for i in range(len(frames)):
        frame = frames[i]
        bpy.context.scene.frame_set(frame)
        print(positionFinalX)
        print(positionFinalY)
        car.positionXYZ[0] = positionFinalX[i]
        car.positionXYZ[1] = positionFinalY[i]
        car.facingAngle = turn[i]
        car.updateOb()
        bpy.ops.anim.keyframe_insert(type='LocRotScale')
    return lastFrame + AI_UPDATE_RATE

    
    
    
if __name__ == "__main__":
    lastFrame = 1
    ob = bpy.data.objects["Voiture"]
    Line0 = bpy.data.objects["Cube0"]
    Line1 = bpy.data.objects["Cube1"]
    Line2 = bpy.data.objects["Cube2"]
    Line3 = bpy.data.objects["Cube3"]
    Line4 = bpy.data.objects["Cube4"]
    car = ObjectCar(ob,Line0,Line1,Line2,Line3,Line4)
    
    car.positionXYZ = [0,0,0.385]
    car.facingAngle = 0
    car.updateOb()
    for i in range(0,50):
        lastFrame = turnAngleSpeed(0,10,lastFrame,car)
        
    print("position de la voiture:" + str(car.positionXYZ))
    print("position du sensor centrale:" + str(car.lineSensor))
