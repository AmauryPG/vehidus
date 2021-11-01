import bpy
from bpy import context as C
from bpy import data as D
import sys
import time 
 
def forward(position, distance, deltatTime):
    vitesse = 7000
    if position <= distance:
        position = position + vitesse * deltatTime 
    return position


if __name__ == "__main__":
    #bpy.ops.mesh.primitive_cube_add()
    #ob = bpy.context.active_object
    ob = bpy.data.objects["Voiture"]
    bpy.ops.object.shade_smooth()

    #set de mouvement
    rotations = [0, 6.3]
    frames = [1, 250] 
    moveY = [0, 2]
    prevTime = time.time() 
    mvX = 0
    mvY = 0

    #boucle d'animation
    for i in range(len(rotations)):
        rotation = rotations[i]
        frame = frames[i]

        curtTime = time.time() 

        bpy.context.scene.frame_set(frame)
        deltatTime = curtTime - prevTime
        ob.location[0] = forward(mvX, 100, deltatTime)
        ob.location[1] = forward(mvY, 2, deltatTime)

        prevTime = curtTime

        bpy.ops.anim.keyframe_insert(type='LocRotScale', confirm_success=True)

    #joue l'animation montrée
    bpy.ops.screen.animation_play()
