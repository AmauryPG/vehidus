'''
Ficher principal pour l'excution du AI dans le monde reel.
'''
from SunFounder_Line_Follower import Line_Follower
import independantWheels
from picar import ADC
import time
import picar
import ai_v2
import stop
import numpy as np

REFERENCES = [105, 105  , 105, 105, 105]

VITESSE_MAX = 60
VITESSE_MIN = 10
ACCELERATION_MAX = 0.1
LOOP_UPDATE_TIME = 1/24

def acceleration(bw, speed, accelerationVoulue):
  if speed < VITESSE_MAX:
    speed = speed + 1
    bw.speed = speed
  return speed

def decceleration_fct(bw, speed, accelerationVoulue):
  for i in range(48): 
    if speed > 30:
      speed = speed - 1
      bw.speed = speed
    time.sleep(LOOP_UPDATE_TIME)
  return speed

if __name__ == "__main__":
  picar.setup()
  lf = Line_Follower.Line_Follower()
  fw = independantWheels.Front_Wheels(db='config')
  bw = independantWheels.Back_Wheels(db='config')
  lf.references = REFERENCES
  done, angle, sens = False, 0, True
  continuer_tournant = False
  sens = True
  attend = False
  currentSpeed = 0
  bw.speed = currentSpeed
  decceleration = False
  while not done : 
    lf_status_now = lf.read_digital()
    done, angle, sens, continuer_tournant = ai_v2.get_wheel_angles(lf_status_now, angle, continuer_tournant, sens)
    fw.turn(90 + angle,currentSpeed,bw)
    currentSpeed = acceleration(bw, currentSpeed, ACCELERATION_MAX)
    if not sens:
      bw.backward()
      currentSpeed=decceleration_fct(bw,currentSpeed,ACCELERATION_MAX)
    else:
      bw.forward()
    print(lf_status_now)
    if continuer_tournant : 
      decceleration = True
    else: 
      decceleration = False
    if continuer_tournant:
      currentSpeed = decceleration_fct(bw, currentSpeed, 102)
    time.sleep(LOOP_UPDATE_TIME)
  lf = Light_Follower.Light_Follower()
  fw = front_wheels.Front_Wheels(db='config')
  bw = back_wheels.Back_Wheels(db='config')
  adc = ADC()
  
  bw.speed = 0
  fw.turn = 90
  