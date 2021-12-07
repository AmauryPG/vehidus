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

CONTINUE = False
ARRETER = True

FORWARD = True
RECULER = False
DISTANCE_ENTRE_CAPTEURS = 0.18
DISTANCE_ENTRE_CENTRE_MASSE_ET_CAPTEUR = 1.41
FACTOR = ((DISTANCE_ENTRE_CAPTEURS/DISTANCE_ENTRE_CENTRE_MASSE_ET_CAPTEUR)/2)*0.6
speed_MAX = 2.5
speed_MIN = 0.75
ACCELERATION_MAX = 1

CONTINUETournant = True
ARRETER_TOURNANT = False

RAYON = 1.4
DISTANCE_BOL_VEHICULE =  0.57316

POS_INITIALE_zBille = 0.66331

class Bille:
    def __init__(self):
        self.obj = bpy.data.objects["Bille"]
        self.obj.animation_data_clear()
        self.positionXYZ = [0,0,0.385]
        self.facingAngle = 0
        self.XYZ = [0, 0, 0]
        self.zBille = [0, 0]
        self.mat = bpy.data.materials.new("PKHG")
        self.mat.diffuse_color=(0,1,0,1)
        self.obj.active_material=self.mat
        self.speed = [0,0]
        self.acceleration = [0, 0]
        
    def mouvBille(self, aCarX, aCarY):
        g = 9.81
        aNx = 0
        aNy = 0

        dt = 1
        # Ajouter un coef de friction
        if self.XYZ[0] == 0:
            aNx = 0
        else:
            aNx = g*self.zBille[0]/(-self.XYZ[0])
            
        if self.XYZ[1] == 0:
            aNy = 0
        else:
            aNy = g*self.zBille[1]/(-self.XYZ[1])
            
        self.acceleration[0] = aNx - aCarX
        self.speed[0] = self.speed[0] + self.acceleration[0]*(dt/FPS)
        self.XYZ[0] = self.XYZ[0] + self.speed[0]*(dt/FPS) + 0.5*self.acceleration[0]*(dt/FPS)**2
        
        self.acceleration[1] = aNy - aCarY
        self.speed[1] = self.speed[1] + self.acceleration[1]*(dt/FPS)
        self.XYZ[1] = self.XYZ[1] + self.speed[1]*(dt/FPS) + 0.5*self.acceleration[1]*(dt/FPS)**2

        self.zBille[0] = RAYON - np.sqrt(RAYON**2-self.XYZ[0]**2)
        self.zBille[1] = RAYON - np.sqrt(RAYON**2-self.XYZ[1]**2)
        z_norm = RAYON - np.sqrt(RAYON**2-(self.XYZ[0]**2+self.XYZ[1]**2))
        
        
def aCentripete(v1x, v1y, v2x, v2y, angle):
    v2 = math.sqrt(v2x**2+v2y**2)
    if v2 == 0.0:
        return [0, 0]
    else:
        r = v2**2*AI_UPDATE_RATE/FPS / (np.sqrt((v2x-v1x)**2+(v2y-v1y)**2))
        a = v2**2/r
        return [a*np.cos(angle), a*np.sin(angle)]

# Return DONE (True or false), Angle de roue en degre (entre -45 et 45), Sens (avant = True, arriere = False), CONTINUE TOURNANT (TRUE OR FALSE)
def getWheelAngles(lineReader, previousValue, ContinueTournant):
    if lineReader == [BLACK,WHITE,WHITE,WHITE,WHITE]:
        return CONTINUE, np.arctan(FACTOR * 4), FORWARD, True
    elif lineReader == [BLACK, BLACK, WHITE, WHITE, WHITE]:
        return CONTINUE, np.arctan(FACTOR * 3), FORWARD, False
    elif lineReader == [WHITE, BLACK, WHITE, WHITE, WHITE]:
        return CONTINUE, np.arctan(FACTOR * 2), FORWARD, False
    elif lineReader == [WHITE, BLACK, BLACK, WHITE, WHITE]:
        return CONTINUE, np.arctan(FACTOR * 1), FORWARD, False
    ## CONTINUING FORWARD
    elif lineReader == [WHITE, WHITE, BLACK, WHITE, WHITE]:
        return CONTINUE, np.arctan(FACTOR * 0), FORWARD, False
    ## TURNING RIGHT
    elif lineReader == [WHITE,WHITE,WHITE,WHITE,BLACK]:
        return CONTINUE, np.arctan(-FACTOR * 4), FORWARD, True
    elif lineReader == [WHITE, WHITE, WHITE, BLACK, BLACK]:
        return CONTINUE, np.arctan(-FACTOR * 3), FORWARD, False
    elif lineReader == [WHITE, WHITE, WHITE, BLACK, WHITE]:
        return CONTINUE, np.arctan(-FACTOR * 2), FORWARD, False
    elif lineReader == [WHITE, WHITE, BLACK, BLACK, WHITE]:
        return CONTINUE, np.arctan(-FACTOR * 1), FORWARD, False
    elif lineReader == [BLACK, BLACK, BLACK, BLACK, BLACK]:
        return ARRETER, np.arctan(FACTOR * 0), FORWARD, False
    else:
        if ContinueTournant:
            return CONTINUE, previousValue, FORWARD, True
        else:
            return CONTINUE, previousValue, FORWARD, False

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
        
    def accelerate(self, speedMax=speed_MAX):
        if self.speed < speedMax:
            self.speed += ACCELERATION_MAX*1/FPS
            
    def decelerate(self, speedMin=speed_MIN):
        if self.speed > speedMin:
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
        
        
        self.sensorsValues = intersectionCheck(self.lineSensorsArray,self.path)
        #self.sensorDist =  distanceCheck(self.distSensor,self.obstacles)
        
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

def intersectionCheck(sensorList,path):
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
            objNowBVHtree = BVHTree.FromBMesh(bm1)
            objNextBVHtree = BVHTree.FromBMesh(bm2)           

            #get intersecting pairs
            inter = objNowBVHtree.overlap(objNextBVHtree)

            #if list is empty, no objects are touching
            if inter != []:
                #print(sensor.name + " and " + path.name +" are intersecting")
                result.append(1)
            else:
                #print(sensor.name + " and " + path.name + " are not intersecting")
                result.append(0)
    print(result)
    return result

def avoidObstacle(distance, nbFrames):
        firstTurn = []
        middleTurn = []
        lastTurn = []
        middleRadius = 20
        firstRadius = 20
        lastRadius = 20
        angle = 0
        
        
# first turn
        circ = 2 * np.pi * firstRadius
        arcAngle = np.tan(distance/firstRadius)
        arcLength = circ/arcAngle
        turningAngle = getTurningAngle(firstRadius)
        for f in range(nbFrames):
            angle -= turningAngle/nbFrames
            firstTurn.append(angle)
            
# second turn
        circ = 2 * np.pi * middleRadius
        arcAngle = np.pi
        arcLength = circ/arcAngle
        turningAngle = getTurningAngle(middleRadius)
        for f in range(2*nbFrames):
            angle += turningAngle/nbFrames
            middleTurn.append(angle)

# last turn
        circ = 2 * np.pi * lastRadius
        arcAngle = np.pi/2
        arcLength = circ/arcAngle
        turningAngle = getTurningAngle(lastRadius)
        for f in range(nbFrames):
            angle -= turningAngle/nbFrames
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

def distanceCheck(sensorDist,Obstacles):
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
        objNowBVHtree = BVHTree.FromBMesh(bm1)
        objNextBVHtree = BVHTree.FromBMesh(bm2)           
        #get intersecting pairs
        inter = objNowBVHtree.overlap(objNextBVHtree)
        #if list is empty, no objects are touching
        if inter != []:
            xdist = Obj.location[0] - sensorDist.location[0]
            ydist = Obj.location[1] - sensorDist.location[1]
            distanceCalculee = np.sqrt(xdist**2 + ydist**2)
            print("Car seeing " + Obj.name + " at " + str(distanceCalculee))
        else:
            #print(sensor.name + " and " + path.name + " are not intersecting")
            distanceCalculee = 100000
            print("Can't see obstacle")
        
        if distanceCalculee < distance:
            distance = distanceCalculee
    
    return distance
    
if __name__ == "__main__":
    #Settings
    done = False
    lastFrame = 1
    path = bpy.data.objects["Path"]
    ob = bpy.data.objects["Car"]
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
    obstacles = [bpy.data.objects["Obstacle1"], bpy.data.objects["Obstacle2"],
                 bpy.data.objects["Obstacle3"], bpy.data.objects["Obstacle4"],
                 bpy.data.objects["Obstacle5"]]
    
    # First Frames
    car = ObjectCar(ob,lineSensorsArray,distSensor,path,obstacles)
    car.updateOb(lastFrame)
    prevValueTournant = 0
    CONTINUETournant = False
    sensorsValues = [0,0,1,0,0]
    isMovingAround = False
    j, obstacleAvoidBuffer = 0, 0
    fiveSec, thirtyCM = False, False
    
    for i in range(5000):
                
            if sensorsValues ==  [1, 1, 1, 1, 1]:
                break
                
            #Get the distance between the sensor and the obstacles
            distance = distanceCheck(distSensor, obstacles)
            if obstacleAvoidBuffer > 2 * FPS  and sensorsValues == [0,0,1,0,0]:
                isMovingAround = False
                j = 0
                obstacleAvoidBuffer = 0
                fiveSec = False
                thirtyCM = False
             
            avoidDistance = car.updateAvoidDistance()
            #Obstacle detection
            if (avoidDistance - 0.05) < distance < (avoidDistance + 0.05) or isMovingAround is True:
                isMovingAround = True

                if j == 0 :
                    backwardAngle = car.facingAngle
                
                if car.speed != 0 and not fiveSec:
                    car.stop()
                elif car.speed == 0 and not fiveSec:
                    # Wait 5s
                    if j == 5*FPS:
                        car.speed = -speed_MIN
                        fiveSec = True
                    else:
                        j += 1
                        
                elif thirtyCM:
                    car.accelerate(2)
                    avoidPath = avoidObstacle(distance, FRAME_AVOID)
                    car.facingAngle = backwardAngle + avoidPath[obstacleAvoidBuffer]
                    obstacleAvoidBuffer += 1

                        
                elif 2.95 < distance < 3.05 and fiveSec:
                    car.speed = 0
                    thirtyCM = True
             
            elif done:
                car.speed = 0
                        
            elif sensorsValues != [0, 0, 1, 0, 0]:
                car.decelerate()
                
            else:
                car.accelerate()
        
            if not done:
                done, prevValueTournant, sens, CONTINUETournant = getWheelAngles(sensorsValues, prevValueTournant, CONTINUETournant)
                
            linAccelerationCar = [(car.speed - car.oldSpeed)/(AI_UPDATE_RATE/FPS)*np.cos(car.facingAngle), (car.speed - car.oldSpeed)/(AI_UPDATE_RATE/FPS)*np.sin(car.facingAngle)]
            lastFrame, sensorsValues, distValue = turnAngleSpeed(prevValueTournant, car.speed, lastFrame, car)
            
            if sensorsValues != [0, 0, 1, 0, 0] and not done:
                if sensorsValues[0] == 1 or sensorsValues[1] == 1:
                    angle = car.facingAngle + 90
                else:
                    angle = car.facingAngle - 90
                
                aCentripeteCar = aCentripete(car.oldSpeed*np.cos(car.oldFacingAngle), car.oldSpeed*np.sin(car.oldFacingAngle),
                                   car.speed*np.cos(car.facingAngle), car.speed*np.sin(car.facingAngle), angle)
                accelerationCar = [linAccelerationCar[0]+ aCentripeteCar[0], linAccelerationCar[1]+ aCentripeteCar[1]]
            
            else:
                accelerationCar = linAccelerationCar
                
            
            # La bille fonctionne en m/s2 et le vehicule en dm/s2
            bille.mouvBille(accelerationCar[0]/10, accelerationCar[1]/10)
            bille.obj.location = [bille.XYZ[0]+car.positionXYZ[0]+DISTANCE_BOL_VEHICULE*np.cos(car.facingAngle),
                             bille.XYZ[1]+car.positionXYZ[1]+DISTANCE_BOL_VEHICULE*np.sin(car.facingAngle),
                             bille.XYZ[2]+POS_INITIALE_zBille]
                              
            if bille.XYZ[2] > 0.15:
                mat = bpy.data.materials.new("PKHG")
                bille.mat.diffuse_color=(1,0,0,1)
                bille.obj.active_material=bille.mat
                print("bille tombee")

                
            car.oldFacingAngle = car.facingAngle
            car.oldSpeed = car.speed
            
                                              
            bille.obj.keyframe_insert(data_path="location", frame=lastFrame)