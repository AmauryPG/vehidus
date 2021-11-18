import bpy
from bpy import context as C
from bpy import data as D
import numpy as np
import sys


def mouv_bille(voiture_moveX, voiture_moveY, a_voiture_x, a_voiture_y, nb_frames):
    g = 9.81
    voiture = bpy.data.objects["Voiture"]
    bille = bpy.data.objects["Bille"]
    
    pos_initiale_x = 0.071198
    pos_initiale_y = 0
    pos_initiale_z = 0.025092
    
    
    rayon = 0.14
    z = 0
    x = 0
    y = 0
    v_x = 0
    v_y = 0
    a_bille_x = 0
    a_bille_y = 0
    a_Nx = 0
    a_Ny = 0
    z_x = 0
    z_y = 0
    bille_Z = []
    bille_X = []
    bille_Y = []
    norme = 0

    dt = 1
    # Ajouter un coef de friction
    # Ajouter la meme chose en y pour le 3D
    for t in range(0,nb_frames):
        if x == 0:
            a_Nx = 0
        else:
            a_Nx = g*z_x/(-x)
            
        if y == 0:
            a_Ny = 0
        else:
            a_Ny = g*z_y/(-y)
            
        a_bille_x = a_Nx - a_voiture_x[t]
        x = x + v_x*(dt/25) + 0.5*a_bille_x*(dt/25)**2
        v_x = v_x + a_bille_x*(dt/25)
        
        a_bille_y = a_Ny - a_voiture_y[t]
        y = y + v_y*(dt/25) + 0.5*a_bille_y*(dt/25)**2
        v_y = v_y + a_bille_y*(dt/25)

        z_x = rayon - np.sqrt(rayon**2-x**2)
        z_y = rayon - np.sqrt(rayon**2-y**2)
        z = rayon - np.sqrt(rayon**2-(x**2+y**2))
        
        bille_X.append(x)
        bille_Z.append(z)
        bille_Y.append(y)
        
        
    print("Max z = ", 1000*max(bille_Z), " mm")
    
    frames = np.arange(0,nb_frames)
    bille_moveX = [voiture_moveX[i]+pos_initiale_x+bille_X[i] for i in range(0, len(voiture_moveX))]
    bille_moveX[0] = pos_initiale_x
    bille_moveY = [voiture_moveY[i]+pos_initiale_y+bille_Y[i] for i in range(0, len(voiture_moveY))]
    bille_moveY[0] = pos_initiale_y
    bille_moveZ = [pos_initiale_z+bille_Z[i] for i in range(0, len(bille_Z))]
    bille_moveZ[0] = pos_initiale_z
    
    #boucle d'animation
    for i in range(len(voiture_moveX)):
        frame = frames[i]
        mvX = voiture_moveX[i]
        mvY = voiture_moveY[i]
        billeX = bille_moveX[i]
        billeY = bille_moveY[i]
        billeZ = bille_moveZ[i]
        bpy.context.scene.frame_set(frame)
        voiture.location[0] = mvX
        voiture.location[1] = mvY
        bille.location[0] = billeX
        bille.location[1] = billeY
        bille.location[2] = billeZ
        x
        bpy.ops.anim.keyframe_insert(type='LocRotScale')

    #joue l'animation montrée
    bpy.ops.screen.animation_play()


#fonction  

nb_frames = int(50)
#À modifier avec l'équation de la voiture
a_voiture_x = [0.2 for _ in range(nb_frames)]
a_voiture_y =  [0.1 for _ in range(nb_frames)]
    
voiture_moveX = [0.5*a_voiture_x[t]*(t/25)**2 for t in range(nb_frames)]
voiture_moveY = [0.5*a_voiture_y[t]*(t/25)**2 for t in range(nb_frames)]

mouv_bille(voiture_moveX, voiture_moveY, a_voiture_x, a_voiture_y, nb_frames)