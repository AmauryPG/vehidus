#!/usr/bin/python
'''
**********************************************************************
* Filename    : ai.py
* Description : fichier principal pour l'AI
* Author      : Vehidus  
* Version     : 3.0
**********************************************************************
'''
import numpy as np 
import math
import time

# constante global
WHITE = 0
BLACK = 1
CONTINUER = False
ARRETER = True
AVANCER = True
RECULER = False
# Le facteur change entre la simulation et la realite
FACTEUR = 45/4
CONTINUER_TOURNANT = True
ARRETER_TOURNANT = False

# But : logique principal de la voiture
'''
Return 
DONE (True or false), 
Angle de roue en degre (entre -45 et 45), 
Sens (avant = True, arriere = False), 
CONTINUER TOURNANT (TRUE OR FALSE)
'''
def get_wheel_angles(lineReader, previousValue, continuerTournant, previousSens):
  # cas de fin de parcours
  if lineReader == [BLACK,BLACK,BLACK,BLACK,BLACK]:
    return ARRETER, -FACTEUR * 0, AVANCER, False
  
  # tournant à gauche
  elif lineReader == [BLACK,WHITE,WHITE,WHITE,WHITE]:
    return CONTINUER, -FACTEUR * 10, AVANCER, True
  elif lineReader == [BLACK, BLACK, WHITE, WHITE, WHITE]:
    return CONTINUER, -FACTEUR * 3, AVANCER, False
  elif lineReader == [WHITE, BLACK, WHITE, WHITE, WHITE]:
    return CONTINUER, -FACTEUR * 2, AVANCER, False
  elif lineReader == [WHITE, BLACK, BLACK, WHITE, WHITE]:
    return CONTINUER, -FACTEUR * 1, AVANCER, False
  
  # continuer droit
  elif lineReader == [WHITE, WHITE, BLACK, WHITE, WHITE]:
    return CONTINUER, FACTEUR * 0, AVANCER, False
  
  # tournant à droit
  elif lineReader == [WHITE,WHITE,WHITE,WHITE,BLACK]:
    return CONTINUER, FACTEUR * 4, AVANCER, True
  elif lineReader == [WHITE, WHITE, WHITE, BLACK, BLACK]:
    return CONTINUER, FACTEUR * 3, AVANCER, False
  elif lineReader == [WHITE, WHITE, WHITE, BLACK, WHITE]:
    return CONTINUER, FACTEUR * 2, AVANCER, False
  elif lineReader == [WHITE, WHITE, BLACK, BLACK, WHITE]:
    return CONTINUER, FACTEUR * 1, AVANCER, False
  else:
    # cas si la voiture perd la ligne 
    if continuerTournant:
      return CONTINUER, previousValue, AVANCER, True
    else:
      return CONTINUER, previousValue, previousSens, False

      