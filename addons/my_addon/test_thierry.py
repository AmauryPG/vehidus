import bpy
from bpy import context as C
from bpy import data as D
import sys
import numpy as np
import addon_utils
import bmesh
from mathutils.bvhtree import BVHTree
import math

import numpy as np
import math

import time
from os import system
#fonction pour clear l'outuput
cls = lambda: system('cls')

BLACK = 0
WHITE = 1

CONTINUER = False
ARRETER = True

AVANCER = True
RECULER = False

FACTEUR = 35/4

CONTINUER_TOURNANT = True
ARRETER_TOURNANT = False
# Return DONE (True or false), Angle de roue en degre (entre -45 et 45), Sens (avant = True, arriere = False), CONTINUER TOURNANT (TRUE OR FALSE)
def get_wheel_angles(line_reader, previous_value, continuer_tournant):
    if line_reader == [WHITE,BLACK,BLACK,BLACK,BLACK]:
        return CONTINUER, -FACTEUR * 4, AVANCER, True
    elif line_reader == [WHITE, WHITE, BLACK, BLACK, BLACK]:
        return CONTINUER, -FACTEUR * 3, AVANCER, False
    elif line_reader == [BLACK, WHITE, BLACK, BLACK, BLACK]:
        return CONTINUER, -FACTEUR * 2, AVANCER, False
    elif line_reader == [BLACK, WHITE, WHITE, BLACK, BLACK]:
        return CONTINUER, -FACTEUR * 1, AVANCER, False
    ## CONTINUER TOUT DROIT
    elif line_reader == [BLACK, BLACK, WHITE, BLACK, BLACK]:
        return CONTINUER, FACTEUR * 0, AVANCER, False
    ## TOURNANT A DROITE
    elif line_reader == [BLACK,BLACK,BLACK,BLACK,WHITE]:
        return CONTINUER, FACTEUR * 4, AVANCER, True
    elif line_reader == [BLACK, BLACK, BLACK, WHITE, WHITE]:
        return CONTINUER, FACTEUR * 3, AVANCER, False
    elif line_reader == [BLACK, BLACK, BLACK, WHITE, BLACK]:
        return CONTINUER, FACTEUR * 2, AVANCER, False
    elif line_reader == [BLACK, BLACK, WHITE, WHITE, BLACK]:
        return CONTINUER, FACTEUR * 1, AVANCER, False
    else:
        if continuer_tournant:
            return CONTINUER, previous_value, AVANCER, True
        else:
            return CONTINUER, previous_value, AVANCER, False
 

FPS = 24 # frame/seconds
AI_UPDATE_RATE = 1 #frame
TIME_BETWEEN_AI_UPDATE = AI_UPDATE_RATE/FPS # seconds
DISTANCE_X_CAPTEUR_LIGNE = 1.41
DISTANCE_Y_CAPTEUR_LIGNE = 0.18
DISTANCE_X_CAPTEUR_DISTANCE = 1.212
DISTANCE_Z_CAPTEUR_DISTANCE = 0.225


class ObjectCar :

    def __init__(self, ob, lineSensorsArray,distSensor,path,obstacles):
        self.ob = ob
        self.lineSensorsArray = lineSensorsArray
        self.distSensor = distSensor
        self.facingAngle =  ob.rotation_euler[2]
        self.positionXYZ = [ob.location[0],ob.location[1],ob.location[2]]
        self.speed = 0
        self.path = path
        self.obstacles = obstacles
        self.sensorsValues = [0,0,0,0,0]
        self.sensorDist = 0
        
    def updateOb(self, frame):
        self.ob.location = [self.positionXYZ[0], self.positionXYZ[1], self.positionXYZ[2]]
        self.ob.rotation_euler[2] = self.facingAngle 
        self.updateSensorPos(frame)
        return self.sensorsValues, self.sensorDist
        
    def updateSensorPos(self, frame):
        # Sensor Ligne
        self.lineSensorsArray[2].location = [(self.ob.location[0]+DISTANCE_X_CAPTEUR_LIGNE*np.cos(self.ob.rotation_euler[2])),
                                self.ob.location[1]+DISTANCE_X_CAPTEUR_LIGNE*np.sin(self.ob.rotation_euler[2]),
                                0]
        self.lineSensorsArray[0].location = [self.lineSensorsArray[2].location[0] - 2 * DISTANCE_Y_CAPTEUR_LIGNE * np.sin(self.ob.rotation_euler[2]),
                              self.lineSensorsArray[2].location[1] + 2 * DISTANCE_Y_CAPTEUR_LIGNE * np.cos(self.ob.rotation_euler[2]),
                              0]             
        self.lineSensorsArray[1].location = [self.lineSensorsArray[2].location[0] - DISTANCE_Y_CAPTEUR_LIGNE * np.sin(self.ob.rotation_euler[2]),
                              self.lineSensorsArray[2].location[1] + DISTANCE_Y_CAPTEUR_LIGNE * np.cos(self.ob.rotation_euler[2]),
                              0]                     
        self.lineSensorsArray[3].location = [self.lineSensorsArray[2].location[0] + DISTANCE_Y_CAPTEUR_LIGNE * np.sin(self.ob.rotation_euler[2]),
                              self.lineSensorsArray[2].location[1] - DISTANCE_Y_CAPTEUR_LIGNE * np.cos(self.ob.rotation_euler[2]),
                              0]                 
        self.lineSensorsArray[4].location = [self.lineSensorsArray[2].location[0] + 2 * DISTANCE_Y_CAPTEUR_LIGNE * np.sin(self.ob.rotation_euler[2]),
                              self.lineSensorsArray[2].location[1] - 2 * DISTANCE_Y_CAPTEUR_LIGNE * np.cos(self.ob.rotation_euler[2]),
                              0]
        for cube in lineSensorsArray:
            cube.rotation_euler[2] = self.ob.rotation_euler[2]
            cube.keyframe_insert(data_path="location", frame=frame)
            cube.keyframe_insert(data_path="rotation_euler", frame=frame)
            
        #Sensor distance
        self.distSensor.location = [(self.ob.location[0]+DISTANCE_X_CAPTEUR_DISTANCE*np.cos(self.ob.rotation_euler[2])),
                                self.ob.location[1]+DISTANCE_X_CAPTEUR_DISTANCE*np.sin(self.ob.rotation_euler[2]),self.ob.location[2] +
                                DISTANCE_Z_CAPTEUR_DISTANCE]
        self.distSensor.rotation_euler[1] = -np.pi/2
        self.distSensor.rotation_euler[0] = self.ob.rotation_euler[2]
        self.distSensor.rotation_euler[2] = 0
        self.distSensor.keyframe_insert(data_path="location", frame=frame)
        self.distSensor.keyframe_insert(data_path="rotation_euler", frame=frame)
        
        
        self.sensorsValues = intersection_check(self.lineSensorsArray,self.path)
        #self.sensorDist =  distance_check(self.distSensor,self.obstacles)
        
def turnAngleSpeed(angle, speed, lastFrame, car):
    angle = - angle
    facingAngle = car.facingAngle
    frames = [lastFrame,  lastFrame + AI_UPDATE_RATE]
    distance = (TIME_BETWEEN_AI_UPDATE) * speed 
    
    newPositionX = car.positionXYZ[0] + np.cos(angle)*distance
    newPositionY = car.positionXYZ[1] + np.sin(angle)*distance
    
    positionFinalX = [car.positionXYZ[0], newPositionX]
    positionFinalY = [car.positionXYZ[1], newPositionY]
    
    if angle == 0:
        turn = [facingAngle, facingAngle]
    else:
        turn = [facingAngle,  facingAngle + np.arctan(newPositionY/newPositionX)] 
    
    print("Frame : ", lastFrame)
    print("position X : ", positionFinalX)
    print("position Y : ", positionFinalY)
    print("angle : ", angle)
    print("facingAngle ", facingAngle) 
    
    sensors = []
    for i in range(len(frames)):
        frame = frames[i]
        bpy.context.scene.frame_set(frame)
        car.positionXYZ[0] = positionFinalX[i]
        car.positionXYZ[1] = positionFinalY[i]
        car.facingAngle = turn[i]
        sensorsValues, distValue = car.updateOb(frame)
        car.ob.keyframe_insert(data_path="location", frame=frame)
        car.ob.keyframe_insert(data_path="rotation_euler", frame=frame)
    return lastFrame + AI_UPDATE_RATE, sensorsValues, distValue

def intersection_check(sensorList,path):
    scene =  bpy.context.scene
    result = []
    #check every object for intersection with every other object
    for sensor in sensorList:
            #create bmesh objects
            bm1 = bmesh.new()
            bm2 = bmesh.new()

            #fill bmesh data from objects
            bm1.from_mesh(scene.objects[sensor.name].data)
            bm2.from_mesh(scene.objects[path.name].data)            

            #fixed it here:
            bm1.transform(scene.objects[sensor.name].matrix_world)
            bm2.transform(scene.objects[path.name].matrix_world) 

            #make BVH tree from BMesh of objects
            obj_now_BVHtree = BVHTree.FromBMesh(bm1)
            obj_next_BVHtree = BVHTree.FromBMesh(bm2)           

            #get intersecting pairs
            inter = obj_now_BVHtree.overlap(obj_next_BVHtree)

            #if list is empty, no objects are touching
            if inter != []:
                #print(sensor.name + " and " + path.name +" are intersecting")
                result.append(1)
            else:
                #print(sensor.name + " and " + path.name + " are not intersecting")
                result.append(0)
    print(result)
    return result

def distance_check(sensorDist,Obstacles):
    scene =  bpy.context.scene
    Distance = 0.0
    #check every object for intersection with every other object
    for Obj in Obstacles:
            #create bmesh objects
            bm1 = bmesh.new()
            bm2 = bmesh.new()

            #fill bmesh data from objects
            bm1.from_mesh(scene.objects[Obj.name].data)
            bm2.from_mesh(scene.objects[sensorDist.name].data)            

            #fixed it here:
            bm1.transform(scene.objects[Obj.name].matrix_world)
            bm2.transform(scene.objects[sensorDist.name].matrix_world) 

            #make BVH tree from BMesh of objects
            obj_now_BVHtree = BVHTree.FromBMesh(bm1)
            obj_next_BVHtree = BVHTree.FromBMesh(bm2)           
            #get intersecting pairs
            inter = obj_now_BVHtree.overlap(obj_next_BVHtree)
            #if list is empty, no objects are touching
            if inter != []:
                xdist = Obj.location[0] - sensorDist.location[0]
                ydist = Obj.location[1] - sensorDist.location[1]
                Distance = np.sqrt(xdist**2 + ydist**2)
                print("Car seeing " + Obj.name + " at " + str(Distance))
            else:
                #print(sensor.name + " and " + path.name + " are not intersecting")
                Distance = 100000
                print("Can't see shit")
    return Distance
    
    
if __name__ == "__main__": 
    cls()
    prevAngle = 0
    #Settings
    lastFrame = 1
    path = bpy.data.objects["Path"]
    ob = bpy.data.objects["Voiture"]
    #obst1 = bpy.data.objects["Cube"]
    #obst2 = bpy.data.objects["Cube2"]
    ob.animation_data_clear()
    lineSensorsArray = [None] * 5
    for i in range(5):
        lineSensorsArray[i] = bpy.data.objects["SensorLine" + str(i)]
        lineSensorsArray[i].animation_data_clear()
        
    distSensor = bpy.data.objects["SensorDist"]
    distSensor.rotation_euler[0] = 0
    distSensor.rotation_euler[1] = -np.pi/2
    distSensor.rotation_euler[2] = 0
    distSensor.animation_data_clear()
    obstacles = []
    
    # First Frames
    car = ObjectCar(ob,lineSensorsArray,distSensor,path,obstacles)
    car.positionXYZ = [0,0,0.385]
    car.facingAngle = 0
    car.updateOb(lastFrame) 
    
    prevValueTournant = 0
    continuerTournant = False
    sensorsValues = [0,0,1,0,0]
    for i in range(10):
        done, prevValueTournant, sens, continuer_tournant = get_wheel_angles(sensorsValues, prevValueTournant, continuerTournant)
        lastFrame, sensorsValues, distValue = turnAngleSpeed(np.deg2rad(prevValueTournant), 2, lastFrame, car)
        #print(i, ' : ' , prevValueTournant)
    
    for i in range(450):
        done, prevValueTournant, sens, continuer_tournant = get_wheel_angles(sensorsValues, prevValueTournant, continuerTournant)
        lastFrame, sensorsValues, distValue = turnAngleSpeed(np.deg2rad(prevValueTournant), 2, lastFrame, car)
        #print(i+10, ' : ' , prevValueTournant)
          
