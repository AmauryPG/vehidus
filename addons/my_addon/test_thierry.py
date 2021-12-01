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
DISTANCE_X_CAPTEUR_LIGNE = 1.2682
DISTANCE_Y_CAPTEUR_LIGNE = 0.18
DISTANCE_X_CAPTEUR_DISTANCE = 1.212
DISTANCE_Z_CAPTEUR_DISTANCE = 0.225
FRAME_AVOID = 40

BLACK = 1
WHITE = 0

CONTINUER = False
ARRETER = True

AVANCER = True
RECULER = False
DISTANCE_ENTRE_CAPTEURS = 0.18
DISTANCE_ENTRE_CENTRE_MASSE_ET_CAPTEUR = 1.41
FACTEUR = ((DISTANCE_ENTRE_CAPTEURS/DISTANCE_ENTRE_CENTRE_MASSE_ET_CAPTEUR)/2)*0.6
VITESSE_MAX = 2.5
VITESSE_MIN = 0.75
ACCELERATION_MAX = 1

CONTINUER_TOURNANT = True
ARRETER_TOURNANT = False

RAYON = 1.4
DISTANCE_BOL_VEHICULE =  0.57316

POS_INITIALE_Z_BILLE = 0.66331

class Bille:
    def __init__(self):
        self.obj = bpy.data.objects["Bille"]
        self.obj.animation_data_clear()
        self.positionXYZ = [0,0,0.385]
        self.facingAngle = 0
        self.XYZ = [0, 0, 0]
        self.z_bille = [0, 0]
        self.mat = bpy.data.materials.new("PKHG")
        self.mat.diffuse_color=(0,1,0,1)
        self.obj.active_material=self.mat
        self.vitesse = [0,0]
        self.acceleration = [0, 0]
        
    def mouv_bille(self, a_voiture_x, a_voiture_y):
        g = 9.81
        a_Nx = 0
        a_Ny = 0

        dt = 1
        # Ajouter un coef de friction
        if self.XYZ[0] == 0:
            a_Nx = 0
        else:
            a_Nx = g*self.z_bille[0]/(-self.XYZ[0])
            
        if self.XYZ[1] == 0:
            a_Ny = 0
        else:
            a_Ny = g*self.z_bille[1]/(-self.XYZ[1])
            
        self.acceleration[0] = a_Nx - a_voiture_x
        self.vitesse[0] = self.vitesse[0] + self.acceleration[0]*(dt/FPS)
        self.XYZ[0] = self.XYZ[0] + self.vitesse[0]*(dt/FPS) + 0.5*self.acceleration[0]*(dt/FPS)**2
        
        self.acceleration[1] = a_Ny - a_voiture_y
        self.vitesse[1] = self.vitesse[1] + self.acceleration[1]*(dt/FPS)
        self.XYZ[1] = self.XYZ[1] + self.vitesse[1]*(dt/FPS) + 0.5*self.acceleration[1]*(dt/FPS)**2

        self.z_bille[0] = RAYON - np.sqrt(RAYON**2-self.XYZ[0]**2)
        self.z_bille[1] = RAYON - np.sqrt(RAYON**2-self.XYZ[1]**2)
        z_norm = RAYON - np.sqrt(RAYON**2-(self.XYZ[0]**2+self.XYZ[1]**2))
        
        
def a_centripete(v1_x, v1_y, v2_x, v2_y, angle):
    v2 = np.sqrt(v2_x**2+v2_y**2)
    r = v2**2*AI_UPDATE_RATE/FPS / (np.sqrt((v2_x-v1_x)**2+(v2_y-v1_y)**2))
    a = v2**2/r
    return [a*np.cos(angle), a*np.sin(angle)]

# Return DONE (True or false), Angle de roue en degre (entre -45 et 45), Sens (avant = True, arriere = False), CONTINUER TOURNANT (TRUE OR FALSE)
def get_wheel_angles(line_reader, previous_value, continuer_tournant):
    if line_reader == [BLACK,WHITE,WHITE,WHITE,WHITE]:
        return CONTINUER, np.arctan(FACTEUR * 4), AVANCER, True
    elif line_reader == [BLACK, BLACK, WHITE, WHITE, WHITE]:
        return CONTINUER, np.arctan(FACTEUR * 3), AVANCER, False
    elif line_reader == [WHITE, BLACK, WHITE, WHITE, WHITE]:
        return CONTINUER, np.arctan(FACTEUR * 2), AVANCER, False
    elif line_reader == [WHITE, BLACK, BLACK, WHITE, WHITE]:
        return CONTINUER, np.arctan(FACTEUR * 1), AVANCER, False
    ## CONTINUER TOUT DROIT
    elif line_reader == [WHITE, WHITE, BLACK, WHITE, WHITE]:
        return CONTINUER, np.arctan(FACTEUR * 0), AVANCER, False
    ## TOURNANT A DROITE
    elif line_reader == [WHITE,WHITE,WHITE,WHITE,BLACK]:
        return CONTINUER, np.arctan(-FACTEUR * 4), AVANCER, True
    elif line_reader == [WHITE, WHITE, WHITE, BLACK, BLACK]:
        return CONTINUER, np.arctan(-FACTEUR * 3), AVANCER, False
    elif line_reader == [WHITE, WHITE, WHITE, BLACK, WHITE]:
        return CONTINUER, np.arctan(-FACTEUR * 2), AVANCER, False
    elif line_reader == [WHITE, WHITE, BLACK, BLACK, WHITE]:
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
        self.facingAngle =  0
        self.positionXYZ = [0,0,0.385]
        self.speed = 0
        self.path = path
        self.obstacles = obstacles
        self.sensorsValues = [0,0,0,0,0]
        self.sensorDist = 0
        self.oldSpeed = 0
        self.oldFacingAngle = 0
        self.avoidDistance = 3.125
        
    def accelerate(self):
        if self.speed < VITESSE_MAX:
            self.speed += ACCELERATION_MAX*1/FPS
            
    def decelerate(self, vitesseMin=VITESSE_MIN):
        if self.speed > vitesseMin:
            self.speed -= ACCELERATION_MAX*1/FPS
            
        if self.speed - ACCELERATION_MAX*1/FPS < 0:
            self.speed = 0
            
    def stop(self):
        self.decelerate(0)
        
    def updateAvoidDistance(self, acceleration=-ACCELERATION_MAX):
        t = -self.speed/acceleration
        self.avoidDistance = 0.5*acceleration*t**2 + self.speed*t + 1
        return self.avoidDistance
        
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
        middleRadius = 17
        firstRadius = 17
        lastRadius = 17
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
    distance = 100000
    distanceCalculee = 0
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
            distanceCalculee = np.sqrt(xdist**2 + ydist**2)
            print("Car seeing " + Obj.name + " at " + str(distanceCalculee))
        else:
            #print(sensor.name + " and " + path.name + " are not intersecting")
            distanceCalculee = 100000
            print("Can't see shit")
        
        if distanceCalculee < distance:
            distance = distanceCalculee
    
    return distance
    
if __name__ == "__main__":
    #Settings
    lastFrame = 1
    path = bpy.data.objects["Path"]
    ob = bpy.data.objects["Voiture"]
    ob.rotation_mode = 'ZXY'
    ob.animation_data_clear()
    lineSensorsArray = [None] * 5
    for i in range(5):
        lineSensorsArray[i] = bpy.data.objects["SensorLine" + str(i)]
        lineSensorsArray[i].animation_data_clear()
        
    distSensor = bpy.data.objects["SensorDist"]
    distSensor.rotation_euler = [0, -np.pi/2, 0]
    distSensor.animation_data_clear()

    bille = Bille()
    obstacles = [bpy.data.objects["Obstacle1"], bpy.data.objects["Obstacle2"]]
    
    # First Frames
    car = ObjectCar(ob,lineSensorsArray,distSensor,path,obstacles)
    car.updateOb(lastFrame)
    prevValueTournant = 0
    continuerTournant = False
    sensorsValues = [0,0,1,0,0]
    isMovingAround = False
    j, obstacleAvoidBuffer = 0, 0
    fiveSec, thirtyCM = False, False
    
    for i in range(1200):
                
            #Get the distance between the sensor and the obstacles
            distance = distance_check(distSensor, obstacles)
            print("car speed : ",car.speed)
            if obstacleAvoidBuffer > 2 * FPS  and sensorsValues == [0,0,1,0,0]:
                isMovingAround = False
                j = 0
                obstacleAvoidBuffer = 0
                fiveSec = False
                thirtyCM = False
                
            avoidDistance = car.updateAvoidDistance()
            print("dist : ", distance)
            print("avoid dist : ", avoidDistance)
            #Obstacle detection
            if (avoidDistance - 0.05) < distance < (avoidDistance + 0.05) or isMovingAround is True:
                isMovingAround = True
                print("if 1------------")

                if j == 0 :
                    backwardAngle = car.facingAngle
                
                if car.speed != 0 and not fiveSec:
                    car.stop()
                elif car.speed == 0 and not fiveSec:
                    # Wait 5s
                    if j == 5*FPS:
                        car.speed = -VITESSE_MIN
                        fiveSec = True
                    else:
                        j += 1
                        
                elif thirtyCM:
                    car.accelerate()
                    avoidPath = avoidObstacle(distance, car.speed, FRAME_AVOID)               
                    car.facingAngle = backwardAngle + avoidPath[obstacleAvoidBuffer]
                    obstacleAvoidBuffer += 1

                        
                elif 2.95 < distance < 3.05 and fiveSec:
                    car.speed = 0
                    thirtyCM = True
                    
            else:
                car.accelerate()
            
            done, prevValueTournant, sens, continuer_tournant = get_wheel_angles(sensorsValues, prevValueTournant, continuerTournant)
            acceleration_lin = [(car.speed - car.oldSpeed)/(AI_UPDATE_RATE/FPS)*np.cos(car.facingAngle), (car.speed - car.oldSpeed)/(AI_UPDATE_RATE/FPS)*np.sin(car.facingAngle)]
            lastFrame, sensorsValues, distValue = turnAngleSpeed(prevValueTournant, car.speed, lastFrame, car)
            
            
            if sensorsValues != [0, 0, 1, 0, 0]:
                if sensorsValues[0] == 1 or sensorsValues[1] == 1:
                    angle = car.facingAngle + 90
                else:
                    angle = car.facingAngle - 90
                
                a_centripete_car = a_centripete(car.oldSpeed*np.cos(car.oldFacingAngle), car.oldSpeed*np.sin(car.oldFacingAngle),
                                   car.speed*np.cos(car.facingAngle), car.speed*np.sin(car.facingAngle), angle)
                a_car = [acceleration_lin[0]+ a_centripete_car[0], acceleration_lin[1]+ a_centripete_car[1]]
            
            else:
                a_car = acceleration_lin
            
            # La bille fonctionne en m/s2 et le vehicule en dm/s2
            bille.mouv_bille(a_car[0]/10, a_car[1]/10)
            bille.obj.location = [bille.XYZ[0]+car.positionXYZ[0]+DISTANCE_BOL_VEHICULE*np.cos(car.facingAngle),
                             bille.XYZ[1]+car.positionXYZ[1]+DISTANCE_BOL_VEHICULE*np.sin(car.facingAngle),
                             bille.XYZ[2]+POS_INITIALE_Z_BILLE]
                              
            if bille.XYZ[2] > 0.015:
                mat = bpy.data.materials.new("PKHG")
                mat.diffuse_color=(1,0,0,1)
                bille.obj.active_material=mat

                
            car.oldFacingAngle = car.facingAngle
            car.oldSpeed = car.speed
            
                                              
            bille.obj.keyframe_insert(data_path="location", frame=lastFrame)