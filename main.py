import bpy
from bpy import context as C
from bpy import data as D
import sys

def main():
    #bpy.ops.mesh.primitive_cube_add()
    #ob = bpy.context.active_object
    ob = bpy.data.objects["Voiture"]
    bpy.ops.object.shade_smooth()

    #set de mouvement
    rotations = [0, 6.3]
    frames = [1, 250]
    moveX = [0, 5]
    moveY = [0, 2]

    #boucle d'animation
    for i in range(len(rotations)):
        rotation = rotations[i]
        frame = frames[i]
        mvX = moveX[i]
        mvY = moveY[i]
        bpy.context.scene.frame_set(frame)
        ob.location[0] = mvX
        ob.location[1] = mvY

        bpy.ops.anim.keyframe_insert(type='LocRotScale')

    #joue l'animation montrée
    bpy.ops.screen.animation_play()

#fonction principal
main()