import bpy
from bpy import context as C
from bpy import data as D
import sys
import numpy as np
import addon_utils
import ObjectCar

FPS = 24
AI_UPDATE_RATE = 5
TIME_BETWEEN_AI_UPDATE = AI_UPDATE_RATE/FPS

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
        print("position des sensors:" + str(car.lineSensor))
    return lastFrame + AI_UPDATE_RATE
    
if __name__ == "__main__":
    lastFrame = 1
    ob = bpy.data.objects["Voiture"]
    car = ObjectCar.ObjectCar(ob)
    
    car.positionXYZ = [0,0,0.385]
    car.facingAngle = 0
    car.updateOb()
    for i in range(0,50):
        lastFrame = turnAngleSpeed(0,10,lastFrame,car)
        
        
