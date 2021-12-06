from SunFounder_Line_Follower import Line_Follower
from picar import front_wheels
from picar import back_wheels
from picar import ADC
import time
import picar
SPEED = 100
TURN = 115
picar.setup()

lf = Line_Follower.Line_Follower()
fw = front_wheels.Front_Wheels(db='config')
bw = back_wheels.Back_Wheels(db='config')
adc = ADC()
REFERENCES = [50, 50, 50, 50, 50]


lf.references = REFERENCES
testStarted = False
bw.speed = SPEED
time_start = None
time_stop = None
while True:
  lt_status_now = lf.read_digital()
  if	lt_status_now == [1,1,1,1,1] and not testStarted:
    fw.turn(TURN)
    time_start = time.time()
    testStarted = True
    print("Test Started")
    time.sleep(1)
  elif lt_status_now == [1,1,1,1,1] and testStarted:
    print("Test Stopped")
    fw.turn(90)
    time_stop = time.time()
    bw.speed = 0
    print("Time elapsed : " +  str((time_stop - time_start)))
    print("Speed : " + str(2 / (time_stop - time_start)) + " m/s")
    break
    