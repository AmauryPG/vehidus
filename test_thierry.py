import bpy
from bpy import context as C
from bpy import data as D
import sys
import numpy as np

FPS = 24
AI_UPDATE_RATE = 5
TIME_BETWEEN_AI_UPDATE = AI_UPDATE_RATE/FPS

def turnAngleSpeed(angle, speed, lastFrame, ob):
    facingAngle = ob.rotation_euler[2]
    turn = [facingAngle, facingAngle + angle]
    frames = [lastFrame,  lastFrame + AI_UPDATE_RATE]
    distance = TIME_BETWEEN_AI_UPDATE * speed
    positionFinalX = [ob.location[0], ob.location[0] + np.cos(facingAngle + angle)*distance]
    positionFinalY = [ob.location[1], ob.location[1] + np.sin(facingAngle + angle)*distance]
    
    for i in range(len(frames)):
        frame = frames[i]
        bpy.context.scene.frame_set(frame)
        ob.location[0] = positionFinalX[i]
        ob.location[1] = positionFinalY[i]
        ob.rotation_euler[2] = turn[i]
        bpy.ops.anim.keyframe_insert(type='LocRotScale')
    return lastFrame + AI_UPDATE_RATE
if __name__ == "__main__":
    lastFrame = 1
    ob = bpy.data.objects["Voiture"]
    ob.rotation_euler[2] = 0 
    ob.location[0] = 0
    ob.location[1] = 0
    for i in range(10):
        lastFrame = turnAngleSpeed(np.pi/10, 10, lastFrame, ob)
    for i in range(10):
        lastFrame = turnAngleSpeed(-np.pi/10, 10, lastFrame, ob)
    for i in range(10):
        lastFrame = turnAngleSpeed(-np.pi/10, 10, lastFrame, ob)
    for i in range(10):
        lastFrame = turnAngleSpeed(np.pi/10, 10, lastFrame, ob)
    