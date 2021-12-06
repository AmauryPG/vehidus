from SunFounder_Light_Follower import Light_Follower
from picar import front_wheels
from picar import back_wheels
from picar import ADC
import time
import picar

picar.setup()

lf = Light_Follower.Light_Follower()
fw = front_wheels.Front_Wheels(db='config')
bw = back_wheels.Back_Wheels(db='config')
adc = ADC()

bw.speed = 0
fw.turn = 90