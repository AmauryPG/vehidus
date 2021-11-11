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
        denum = captLine[0] + captLine[1]+ captLine[2]+ captLine[3]+ captLine[4]

        #s'assure qu'il n'y a pas de division par 0
        if denum == 0:
            denum = 1

        return (captLine[0] + 2 * captLine[1]+ 3 * captLine[2]+ 4 * captLine[3]+ 5 * captLine[4])/denum

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
            angleModifier = np.tan([1 / deltaLine])

            #transforme la valeur de correction négative si on va à gauche
            if deltaLine < 0:
                angleModifier = -np.abs(angleModifier[0])
            else :
                angleModifier = angleModifier[0]

            if self.buffer > 0 :
                #self.angle += angleModifier
                #modulo considérant les nombres négative
                #arrondi l'angle pour réduire un possible surcorrection de l'angle
                self.angle = round(np.pi * math.trunc(angleModifier / np.pi), 2)
            else:
                self.buffer -= 1
        else:
            self.speed = speed
            #peut être cette ligne (85) n'est pas nécessaire
            self.angle = 0

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
    facingAngle = car.facingAngle
    turn = [facingAngle, facingAngle + angle]
    frames = [lastFrame,  lastFrame + AI_UPDATE_RATE]
    distance = (TIME_BETWEEN_AI_UPDATE) * speed
    positionFinalX = [car.positionXYZ[0], car.positionXYZ[0] + np.cos(facingAngle + angle)*distance]
    positionFinalY = [car.positionXYZ[1],car.positionXYZ[1] + np.sin(facingAngle + angle)*distance]
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
    ai = AI(1)
    for i in range(10):
        lastFrame, sensorsValues, distValue = turnAngleSpeed(ai.getAngle(), ai.getSpeed(), lastFrame, car)

    
    for i in range(400):
        lastFrame, sensorsValues, distValue = turnAngleSpeed(ai.getAngle()/(20), ai.getSpeed(), lastFrame, car)
        ai.lineFollower(sensorsValues, 2)
