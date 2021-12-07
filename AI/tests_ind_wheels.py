from SunFounder_Line_Follower import Line_Follower
import independantWheels
from picar import ADC
import time
import picar
SPEED = 100
picar.setup()

lf = Line_Follower.Line_Follower()
fw = independantWheels.Front_Wheels(db='config')
bw = independantWheels.Back_Wheels(db='config')
adc = ADC()
REFERENCES = [50, 50, 50, 50, 50]

lf.references = REFERENCES
testStarted = False
bw.speed = SPEED
time_start = None
time_stop = None

for angle in [40, 50,60,70,80,90,100,110,120,130,140]:
    print('angle:' + str(angle))
    fw.turn(angle,100,bw)
    time.sleep(2)
bw.speed = 0

    