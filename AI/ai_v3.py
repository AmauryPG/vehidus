import numpy as np 
import math
import time
WHITE = 0
BLACK = 1


CONTINUER = False
ARRETER = True


AVANCER = True
RECULER = False

FACTEUR = 45/4

CONTINUER_TOURNANT = True
ARRETER_TOURNANT = False
# Return DONE (True or false), Angle de roue en degre (entre -45 et 45), Sens (avant = True, arriere = False), CONTINUER TOURNANT (TRUE OR FALSE)
def get_wheel_angles(line_reader, previous_value, continuer_tournant, previous_sens):
  # CAS DE FIN
  if line_reader == [BLACK,BLACK,BLACK,BLACK,BLACK]:
    return ARRETER, -FACTEUR * 4, AVANCER, False
  elif line_reader == [BLACK,WHITE,WHITE,WHITE,WHITE]:
    return CONTINUER, -FACTEUR * 4, AVANCER, True
  elif line_reader == [BLACK, BLACK, WHITE, WHITE, WHITE]:
    return CONTINUER, -FACTEUR * 3, AVANCER, False
  elif line_reader == [WHITE, BLACK, WHITE, WHITE, WHITE]:
    return CONTINUER, -FACTEUR * 2, AVANCER, False
  elif line_reader == [WHITE, BLACK, BLACK, WHITE, WHITE]:
    return CONTINUER, -FACTEUR * 1, AVANCER, False
  ## CONTINUER TOUT DROIT
  elif line_reader == [WHITE, WHITE, BLACK, WHITE, WHITE]:
    return CONTINUER, FACTEUR * 0, AVANCER, False
  ## TOURNANT A DROITE
  elif line_reader == [WHITE,WHITE,WHITE,WHITE,BLACK]:
    return CONTINUER, FACTEUR * 4, AVANCER, True
  elif line_reader == [WHITE, WHITE, WHITE, BLACK, BLACK]:
    return CONTINUER, FACTEUR * 3, AVANCER, False
  elif line_reader == [WHITE, WHITE, WHITE, BLACK, WHITE]:
    return CONTINUER, FACTEUR * 2, AVANCER, False
  elif line_reader == [WHITE, WHITE, BLACK, BLACK, WHITE]:
    return CONTINUER, FACTEUR * 1, AVANCER, False
  else:
    if continuer_tournant:
      return CONTINUER, previous_value, RECULER, True
    else:
      return CONTINUER, previous_value, previous_sens, False

      