import bpy
from bpy import context as C
from bpy import data as D
import sys

def avancer_distance_temps(distance, temps) :
    #bpy.ops.mesh.primitive_cube_add()
    #ob = bpy.context.active_object
    ob = bpy.data.objects["Voiture"]
    bpy.ops.object.shade_smooth()

    #set de mouvement
    frames = [1, temps*24]
    moveX = [0, distance]
    moveY = [0, 0]

    #boucle d'animation
    for i in range(len(frames)):
        frame = frames[i]
        mvX = moveX[i]
        mvY = moveY[i]
        bpy.context.scene.frame_set(frame)
        ob.location[0] = mvX
        ob.location[1] = mvY

        bpy.ops.anim.keyframe_insert(type='LocRotScale')

if __name__ == "__main__":
    avancer_distance_temps(20, 5)
    