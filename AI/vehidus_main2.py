from SunFounder_Line_Follower import Line_Follower
from SunFounder_Ultrasonic_Avoidance import Ultrasonic_Avoidance
import independantWheels
from picar import ADC
import time
import picar
import ai_v3
import numpy as np

REFERENCES = [100, 100  , 100, 100, 100]

VITESSE_MAX = 60
VITESSE_MIN = 30
ACCELERATION_MAX = 2
LOOP_UPDATE_TIME = 1/24

def acceleration_graduelle(bw, speed, accelerationVoulue):
  if speed < 60:
    speed = speed + accelerationVoulue
    bw.speed = speed
    time.sleep(LOOP_UPDATE_TIME*2)
  return speed

def acceleration(bw, speed, accelerationVoulue):
  while speed < 60:
    speed = speed + accelerationVoulue
    bw.speed = speed
    time.sleep(LOOP_UPDATE_TIME)
  return speed

def decceleration_fct(bw, speed, accelerationVoulue):
  while speed > 30:
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
  currentSpeed = VITESSE_MIN
  bw.speed = currentSpeed 
  bw.forward()
  ua = Ultrasonic_Avoidance.Ultrasonic_Avoidance(20)
  currentlyAvoidObject = False
  while not done : 
    distanceObjet = ua.get_distance(mount = 5)
    print("Distance  + " + str(distanceObjet))
    if distanceObjet > 20 : 
      currentSpeed = acceleration_graduelle(bw, currentSpeed, ACCELERATION_MAX)
      lf_status_now = lf.read_digital()
      done, angle, sens, continuer_tournant = ai_v3.get_wheel_angles(lf_status_now, angle, continuer_tournant, sens)
      fw.turn(90 + angle,currentSpeed,bw)
      time.sleep(LOOP_UPDATE_TIME)
    else:
      currentlyAvoidObject = True
      while currentlyAvoidObject :
        distanceObjet = ua.get_distance(mount = 5)
        if distanceObjet > 9 :
          currentSpeed = decceleration_fct(bw, currentSpeed, ACCELERATION_MAX)
        else : 
          bw.speed = 0
          fw.turn(90, currentSpeed, bw)
          bw.backward()
          time.sleep(5)
          while distanceObjet < 30:
            distanceObjet = ua.get_distance(mount = 10)
            currentSpeed = acceleration(bw, currentSpeed, ACCELERATION_MAX)

          bw.forward()
          currentSpeed = 0
          bw.speed = currentSpeed
          time.sleep(2)
          fw.turn(90 + 25, currentSpeed, bw)
          currentSpeed = acceleration(bw, currentSpeed, ACCELERATION_MAX)
          time.sleep(2)
          fw.turn(90 - 10, currentSpeed, bw)
          lf_status_now = lf.read_digital()
          time.sleep(1)
          fw.turn(90 - 39, currentSpeed, bw)
          while(lf_status_now[0] != 1):
            lf_status_now = lf.read_digital()
            print(lf.read_digital())
            pass
          print(lf.read_digital())
          currentlyAvoidObject = False
  bw.speed = 0
  fw.turn = 90
  
  
#    if not sens :
#      currentSpeed=decceleration_fct(bw,currentSpeed,ACCELERATION_MAX)
#      bw.backward()
#      if angle < 0:
#        sensor_index = 0
#      else :
#        sensor_index = 4
#      while lf_status_now[sensor_index] == 0:
#        lf_status_now = lf.read_digital()
#        print("lol")
#        time.sleep(LOOP_UPDATE_TIME)
#      bw.forward()
#      fw.turn(90 + 0, currentSpeed, bw)
#      while lf_status_now[2] == 0:
#        lf_status_now = lf.read_digital()
#        currentSpeed = acceleration(bw, currentSpeed, ACCELERATION_MAX)
#    else:
#      #currentSpeed=decceleration_fct(bw,currentSpeed,ACCELERATION_MAX)
#      bw.forward()
    
  