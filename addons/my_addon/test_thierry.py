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


FPS = 24 # frame/seconds
AI_UPDATE_RATE = 1 #frame
TIME_BETWEEN_AI_UPDATE = AI_UPDATE_RATE/FPS # seconds
DISTANCE_X_CAPTEUR_LIGNE = 1.41
DISTANCE_Y_CAPTEUR_LIGNE = 0.18
DISTANCE_X_CAPTEUR_DISTANCE = 1.212
DISTANCE_Z_CAPTEUR_DISTANCE = 0.225
FRAME_AVOID = 75

BLACK = 0
WHITE = 1 

CONTINUER = False
ARRETER = True

AVANCER = True
RECULER = False
distance_entre_capteurs = 0.18
distance_entre_centre_masse_et_capteur = 1.41
FACTEUR = ((distance_entre_capteurs/distance_entre_centre_masse_et_capteur)/2)*0.6

CONTINUER_TOURNANT = True
ARRETER_TOURNANT = False

rayon = 1.4
distance_bol_vehicule =  0.57316
pos_initiale_y = 0
pos_initiale_z = 0.66331

def mouv_bille(a_voiture_x, a_voiture_y, pos_bille, vitesse_bille, z):
    g = 9.81
    
    x = pos_bille[0]
    y = pos_bille[1]
    print(x, y, z)
    v_x = vitesse_bille[0]
    v_y = vitesse_bille[1]
    a_Nx = 0
    a_Ny = 0

    dt = 1
    # Ajouter un coef de friction
    if x == 0:
        a_Nx = 0
    else:
        a_Nx = g*z[0]/(-x)
        
    if y == 0:
        a_Ny = 0
    else:
        a_Ny = g*z[1]/(-y)
        
    a_bille_x = a_Nx - a_voiture_x
    v_x = v_x + a_bille_x*(dt/FPS)
    x = x + v_x*(dt/FPS) + 0.5*a_bille_x*(dt/FPS)**2
    
    a_bille_y = a_Ny - a_voiture_y
    v_y = v_y + a_bille_y*(dt/FPS)
    y = y + v_y*(dt/FPS) + 0.5*a_bille_y*(dt/FPS)**2

    z[0] = rayon - np.sqrt(rayon**2-x**2)
    z[1] = rayon - np.sqrt(rayon**2-y**2)
    z_norm = rayon - np.sqrt(rayon**2-(x**2+y**2))

    return [x, y, z_norm], [v_x, v_y], z


def a_centripete(v1_x, v1_y, v2_x, v2_y, angle):
    v2 = np.sqrt(v2_x**2+v2_y**2)
    r = v2**2*AI_UPDATE_RATE/FPS / (np.sqrt((v2_x-v1_x)**2+(v2_y-v1_y)**2))
    a = v2**2/r
    return [a*np.cos(angle), a*np.sin(angle)]



# Return DONE (True or false), Angle de roue en degre (entre -45 et 45), Sens (avant = True, arriere = False), CONTINUER TOURNANT (TRUE OR FALSE)
def get_wheel_angles(line_reader, previous_value, continuer_tournant):
    if line_reader == [WHITE,BLACK,BLACK,BLACK,BLACK]:
        return CONTINUER, np.arctan(FACTEUR * 4), AVANCER, True
    elif line_reader == [WHITE, WHITE, BLACK, BLACK, BLACK]:
        return CONTINUER, np.arctan(FACTEUR * 3), AVANCER, False
    elif line_reader == [BLACK, WHITE, BLACK, BLACK, BLACK]:
        return CONTINUER, np.arctan(FACTEUR * 2), AVANCER, False
    elif line_reader == [BLACK, WHITE, WHITE, BLACK, BLACK]:
        return CONTINUER, np.arctan(FACTEUR * 1), AVANCER, False
    ## CONTINUER TOUT DROIT
    elif line_reader == [BLACK, BLACK, WHITE, BLACK, BLACK]:
        return CONTINUER, np.arctan(FACTEUR * 0), AVANCER, False
    ## TOURNANT A DROITE
    elif line_reader == [BLACK,BLACK,BLACK,BLACK,WHITE]:
        return CONTINUER, np.arctan(-FACTEUR * 4), AVANCER, True
    elif line_reader == [BLACK, BLACK, BLACK, WHITE, WHITE]:
        return CONTINUER, np.arctan(-FACTEUR * 3), AVANCER, False
    elif line_reader == [BLACK, BLACK, BLACK, WHITE, BLACK]:
        return CONTINUER, np.arctan(-FACTEUR * 2), AVANCER, False
    elif line_reader == [BLACK, BLACK, WHITE, WHITE, BLACK]:
        return CONTINUER, np.arctan(-FACTEUR * 1), AVANCER, False
    else:
        if continuer_tournant:
            return CONTINUER, previous_value, AVANCER, True
        else:
            return CONTINUER, previous_value, AVANCER, False
 


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

def avoidObstacle(distance, speed, nbFrames):
        firstTurn = []
        middleTurn = []
        lastTurn = []
        middleRadius = 17.00
        firstRadius = 17.00
        lastRadius = 17.00
        angle = 0
# first turn
        circ = 2 * np.pi * firstRadius
        arcAngle = np.tan(distance/firstRadius)
        arcLength = circ/arcAngle
        turningAngle = getTurningAngle(firstRadius)
        for f in range(nbFrames):
            angle += turningAngle/nbFrames
            firstTurn.append(angle)
            
# second turn
        circ = 2 * np.pi * middleRadius
        arcAngle = np.pi
        arcLength = circ/arcAngle
        turningAngle = getTurningAngle(middleRadius)
        for f in range(2*nbFrames):
            angle -= turningAngle/nbFrames
            middleTurn.append(angle)

# last turn
        circ = 2 * np.pi * lastRadius
        arcAngle = np.pi/2
        arcLength = circ/arcAngle
        turningAngle = getTurningAngle(lastRadius)
        for f in range(nbFrames):
            angle += turningAngle/nbFrames
            lastTurn.append(angle)
            
        return firstTurn + middleTurn + lastTurn


def getTurningRadius(angle):
    wheelToWheelLength = 13.80
   # wheelToWheelWidth = 13.7 - 2.6
    if angle < 55*0.01745329 or  angle > 135*0.01745329 or angle == 90*0.01745329:
        return 0
    else:
        angle = 90*0.01745329 - angle
        TurningRadius = wheelToWheelLength / np.tan(angle)
        return TurningRadius

def getTurningAngle(radius):

    wheelToWheelLength = 13.80
    angle = np.arctan(wheelToWheelLength/ radius)
    return angle

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
    #Settings
    lastFrame = 1
    path = bpy.data.objects["Path"]
    ob = bpy.data.objects["Voiture"]
    obst1 = bpy.data.objects["Cube"]
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
    bille = bpy.data.objects["Bille"]
    obstacles = [obst1]
    bille.location = [0,0,0]
    bille.animation_data_clear()
    bille_XYZ = bille.location
    # First Frames
    car = ObjectCar(ob,lineSensorsArray,distSensor,path,obstacles)
    car.positionXYZ = [0,0,0.385]
    car.facingAngle = 0
    car.updateOb(lastFrame) 
    isMovingAround = False
    prevAngle = 0
    prevValueTournant = 0
    continuerTournant = False
    sensorsValues = [0,0,1,0,0]
    oldSpeed = 0
    oldSpeedBille = [0,0]
    newSpeedBille = [0,0]
    oldAcceleration = 0
    newAcceleration = 0
    z_bille = [0,0]
    j=0
    
    for i in range(3000):
        if i < 100:
            newSpeed = 0.01*i
        
        distance = distance_check(distSensor, obstacles)
        done, prevValueTournant, sens, continuer_tournant = get_wheel_angles(sensorsValues, prevValueTournant, continuerTournant)
        car.speed = newSpeed
        acceleration_lin = [(newSpeed - oldSpeed)/(AI_UPDATE_RATE/FPS)*np.cos(car.facingAngle), (newSpeed - oldSpeed)/(AI_UPDATE_RATE/FPS)*np.sin(car.facingAngle)]
        lastFrame, sensorsValues, distValue = turnAngleSpeed(prevValueTournant, newSpeed, lastFrame, car)
        
        
        if sensorsValues != [0, 0, 1, 0, 0]:
            if sensorsValues[0] == 1 or sensorsValues[1] == 1:
                angle = car.facingAngle + 90
            else:
                angle = car.facingAngle - 90
            
            a_centripete_car = a_centripete(oldSpeed*np.cos(oldAngle), oldSpeed*np.sin(oldAngle),
                               newSpeed*np.cos(car.facingAngle), newSpeed*np.sin(car.facingAngle), angle)
            a_car = [acceleration_lin[0]+ a_centripete_car[0], acceleration_lin[1]+ a_centripete_car[1]]
        
        else:
            a_car = acceleration_lin
        
        bille_XYZ, oldSpeedBille, z_bille = mouv_bille(a_car[0], a_car[1], [bille_XYZ[0], bille_XYZ[1], bille_XYZ[2]], oldSpeedBille, z_bille)
        bille.location = [bille_XYZ[0]+car.positionXYZ[0]+distance_bol_vehicule*np.cos(car.facingAngle),
                          bille_XYZ[1]+car.positionXYZ[1]+distance_bol_vehicule*np.sin(car.facingAngle),
                          bille_XYZ[2]+pos_initiale_z]
                          
        if bille_XYZ[2] > 0.015:
            mat2 = bpy.data.materials.new("PKHG")
            mat2.diffuse_color=(1,0,0,1)
            bille.active_material=mat2
        if j > 2 * FRAME_AVOID and sensorsValues == [0,0,1,0,0]:
            isMovingAround = False
            j = 0
            
        if distance > 3.144 and distance < 4 or isMovingAround is True:
            if j == 0 :
                previousAngle = car.facingAngle
            avoidPath = avoidObstacle(distance, newSpeed, FRAME_AVOID)
            isMovingAround = True               
            car.facingAngle = previousAngle + avoidPath[j]
            j += 1
            if j == 4*FRAME_AVOID - 1:
                isMovingAround = False
                j = 0 

            
    
        
        oldAngle = car.facingAngle
        oldSpeed = newSpeed
        
                                          
        bille.keyframe_insert(data_path="location", frame=lastFrame)
        oldAcceleration = acceleration_lin
          
