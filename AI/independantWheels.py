#!/usr/bin/env python
'''
**********************************************************************
* Filename    : front_wheels
* Description : A module to control the front wheels of RPi Car
* Author      : Cavon
* Brand       : SunFounder
* E-mail      : service@sunfounder.com
* Website     : www.sunfounder.com
* Update      : Cavon    2016-09-13    New release
*               Cavon    2016-11-04    fix for submodules
**********************************************************************
'''
import Servo
import filedb
import TB6612
import PCA9685
import numpy as np
import time

class Front_Wheels(object):
    ''' Front wheels control class '''
    FRONT_WHEEL_CHANNEL = 0

    _DEBUG = False
    _DEBUG_INFO = 'DEBUG "front_wheels.py":'

    def __init__(self, debug=False, db="config", bus_number=1, channel=FRONT_WHEEL_CHANNEL):
        ''' setup channels and basic stuff '''
        self.db = filedb.fileDB(db=db)
        self._channel = channel
        self._straight_angle = 90
        self.turning_max = 45
        self._turning_offset = int(self.db.get('turning_offset', default_value=0))

        self.wheel = Servo.Servo(self._channel, bus_number=bus_number, offset=self.turning_offset)
        self.debug = debug
        self._debug_('Front wheel PWM channel: %s' % self._channel)
        self._debug_('Front wheel offset value: %s ' % self.turning_offset)

        self._angle = {"left":self._min_angle, "straight":self._straight_angle, "right":self._max_angle}
        self._debug_('left angle: %s, straight angle: %s, right angle: %s' % (self._angle["left"], self._angle["straight"], self._angle["right"]))

    def _debug_(self,message):
        if self._DEBUG:
            print(self._DEBUG_INFO,message)

    def turn_left(self):
        ''' Turn the front wheels left '''
        self._debug_("Turn left")
        self.wheel.write(self._angle["left"])

    def turn_straight(self):
        ''' Turn the front wheels back straight '''
        self._debug_("Turn straight")
        self.wheel.write(self._angle["straight"])

    def turn_right(self):
        ''' Turn the front wheels right '''
        self._debug_("Turn right")
        self.wheel.write(self._angle["right"])

    def turn(self, angle,speed, bw):
        ''' Turn the front wheels to the giving angle '''
        turning_offset_right = 33
        turning_offset_left = 0
        self._debug_("Turn to %s " % angle)
        if angle < self._angle["left"]:
            angle = self._angle["left"]
        if angle > self._angle["right"]:
            angle = self._angle["right"]
        self.wheel.write(angle)
        ratio, backward = getTurningRadiusRatio(angle)
        if ratio < 1.0:
            self._turning_offset = turning_offset_left
            if backward:
                bw.left_wheel.backward()
            else: 
                bw.left_wheel.forward()
            bw.right_wheel.speed = int(speed*ratio)
            bw.left_wheel.speed = int(speed)  
        elif ratio >  1.0:
            self._turning_offset = turning_offset_right
            if backward:
              bw.right_wheel.backward()
            else: 
              bw.right_wheel.forward()
            bw.right_wheel.speed = int(speed)
            bw.left_wheel.speed = int(speed*(1/ratio))
        else:
            self._turning_offset = turning_offset_left
            bw.right_wheel.speed = int(speed)
            bw.left_wheel.speed = int(speed)
    @property
    def channel(self):
        return self._channel
    @channel.setter
    def channel(self, chn):
        self._channel = chn

    @property
    def turning_max(self):
        return self._turning_max

    @turning_max.setter
    def turning_max(self, angle):
        self._turning_max = angle
        self._min_angle = self._straight_angle - angle
        self._max_angle = self._straight_angle + angle
        self._angle = {"left":self._min_angle, "straight":self._straight_angle, "right":self._max_angle}

    @property
    def turning_offset(self):
        return self._turning_offset

    @turning_offset.setter
    def turning_offset(self, value):
        if not isinstance(value, int):
            raise TypeError('"turning_offset" must be "int"')
        self._turning_offset = value
        self.db.set('turning_offset', value)
        self.wheel.offset = value
        self.turn_straight()

    @property
    def debug(self):
        return self._DEBUG
    @debug.setter
    def debug(self, debug):
        ''' Set if debug information shows '''
        if debug in (True, False):
            self._DEBUG = debug
        else:
            raise ValueError('debug must be "True" (Set debug on) or "False" (Set debug off), not "{0}"'.format(debug))

        if self._DEBUG:
            print(self._DEBUG_INFO, "Set debug on")
            print(self._DEBUG_INFO, "Set wheel debug on")
            self.wheel.debug = True
        else:
            print(self._DEBUG_INFO, "Set debug off")
            print(self._DEBUG_INFO, "Set wheel debug off")
            self.wheel.debug = False

    def ready(self):
        ''' Get the front wheels to the ready position. '''
        self._debug_('Turn to "Ready" position')
        self.wheel.offset = self.turning_offset
        self.turn_straight()

    def calibration(self):
        ''' Get the front wheels to the calibration position. '''
        self._debug_('Turn to "Calibration" position')
        self.turn_straight()
        self.cali_turning_offset = self.turning_offset

    def cali_left(self):
        ''' Calibrate the wheels to left '''
        self.cali_turning_offset -= 1
        self.wheel.offset = self.cali_turning_offset
        self.turn_straight()

    def cali_right(self):
        ''' Calibrate the wheels to right '''
        self.cali_turning_offset += 1
        self.wheel.offset = self.cali_turning_offset
        self.turn_straight()

    def cali_ok(self):
        ''' Save the calibration value '''
        self.turning_offset = self.cali_turning_offset
        self.db.set('turning_offset', self.turning_offset)

def test(chn=0):
    import time
    front_wheels = Front_Wheels(channel=chn)
    try:
        while True:
            print("turn_left")
            front_wheels.turn_left()
            time.sleep(1)
            print("turn_straight")
            front_wheels.turn_straight()
            time.sleep(1)
            print("turn_right")
            front_wheels.turn_right()
            time.sleep(1)
            print("turn_straight")
            front_wheels.turn_straight()
            time.sleep(1)
    except KeyboardInterrupt:
        front_wheels.turn_straight()

class Back_Wheels(object):
    ''' Back wheels control class '''
    Motor_A = 17
    Motor_B = 27

    PWM_A = 4
    PWM_B = 5

    _DEBUG = False
    _DEBUG_INFO = 'DEBUG "back_wheels.py":'

    def __init__(self, debug=False, bus_number=1, db="config"):
        ''' Init the direction channel and pwm channel '''
        self.forward_A = True
        self.forward_B = True

        self.db = filedb.fileDB(db=db)

        self.forward_A = int(self.db.get('forward_A', default_value=1))
        self.forward_B = int(self.db.get('forward_B', default_value=1))

        self.left_wheel = TB6612.Motor(self.Motor_A, offset=self.forward_A)
        self.right_wheel = TB6612.Motor(self.Motor_B, offset=self.forward_B)

        self.pwm = PCA9685.PWM(bus_number=bus_number)
        def _set_a_pwm(value):
            pulse_wide = int(self.pwm.map(value, 0, 100, 0, 4095))
            self.pwm.write(self.PWM_A, 0, pulse_wide)

        def _set_b_pwm(value):
            pulse_wide = int(self.pwm.map(value, 0, 100, 0, 4095))
            self.pwm.write(self.PWM_B, 0, pulse_wide)

        self.left_wheel.pwm  = _set_a_pwm
        self.right_wheel.pwm = _set_b_pwm

        self._speed = 0
        self._speed_left = 0
        self._speed_rigth = 0
        self.debug = debug
        self._debug_('Set left wheel to #%d, PWM channel to %d' % (self.Motor_A, self.PWM_A))
        self._debug_('Set right wheel to #%d, PWM channel to %d' % (self.Motor_B, self.PWM_B))
    def set_a_pwm(value):
        pulse_wide = int(self.pwm.map(value, 0, 100, 0, 4095))
        self.pwm.write(self.PWM_A, 0, pulse_wide)

    def set_b_pwm(value):
        pulse_wide = int(self.pwm.map(value, 0, 100, 0, 4095))
        self.pwm.write(self.PWM_B, 0, pulse_wide)
        
    def _debug_(self,message):
        if self._DEBUG:
            print(self._DEBUG_INFO,message)

    def forward(self):
        ''' Move both wheels forward '''
        self.left_wheel.forward()
        self.right_wheel.forward()
        self._debug_('Running forward')

    def backward(self):
        ''' Move both wheels backward '''
        self.left_wheel.backward()
        self.right_wheel.backward()
        self._debug_('Running backward')

    def stop(self):
        ''' Stop both wheels '''
        self.left_wheel.stop()
        self.right_wheel.stop()
        self._debug_('Stop')

    @property
    def speed(self, speed):
        return self._speed

    def get_speed_left(self):
        return self._speed_left

    def get_speed_rigth(self):
        return self._speed_rigth

    @speed.setter
    def speed(self, speed):
        self._speed = speed
        ''' Set moving speeds '''
        self.left_wheel.speed = self._speed
        self.right_wheel.speed = self._speed
        self._debug_('Set speed to %s' % self._speed)

    def speed_left(self, speed):
        self._speed_left = speed
        ''' Set moving speeds '''
        self.left_wheel.speed = self._speed_left
        self._debug_('Set left speed to %s' % self._speed)

    def speed_rigth(self, speed):
        self._speed_rigth = speed
        ''' Set moving speeds '''
        self.right_wheel.speed = self._speed_rigth
        self._debug_('Set right speed to %s' % self._speed)

    @property
    def debug(self):
        return self._DEBUG

    @debug.setter
    def debug(self, debug):
        ''' Set if debug information shows '''
        if debug in (True, False):
            self._DEBUG = debug
        else:
            raise ValueError('debug must be "True" (Set debug on) or "False" (Set debug off), not "{0}"'.format(debug))

        if self._DEBUG:
            print(self._DEBUG_INFO, "Set debug on")
            self.left_wheel.debug = True
            self.right_wheel.debug = True
            self.pwm.debug = True
        else:
            print(self._DEBUG_INFO, "Set debug off")
            self.left_wheel.debug = False
            self.right_wheel.debug = False
            self.pwm.debug = False

    def ready(self):
        ''' Get the back wheels to the ready position. (stop) '''
        self._debug_('Turn to "Ready" position')
        self.left_wheel.offset = self.forward_A
        self.right_wheel.offset = self.forward_B
        self.stop()

    def calibration(self):
        ''' Get the front wheels to the calibration position. '''
        self._debug_('Turn to "Calibration" position')
        self.speed = 50
        self.forward()
        self.cali_forward_A = self.forward_A
        self.cali_forward_B = self.forward_B

    def cali_left(self):
        ''' Reverse the left wheels forward direction in calibration '''
        self.cali_forward_A = (1 + self.cali_forward_A) & 1
        self.left_wheel.offset = self.cali_forward_A
        self.forward()

    def cali_right(self):
        ''' Reverse the right wheels forward direction in calibration '''
        self.cali_forward_B = (1 + self.cali_forward_B) & 1
        self.right_wheel.offset = self.cali_forward_B
        self.forward()

    def cali_ok(self):
        ''' Save the calibration value '''
        self.forward_A = self.cali_forward_A
        self.forward_B = self.cali_forward_B
        self.db.set('forward_A', self.forward_A)
        self.db.set('forward_B', self.forward_B)
        self.stop()

def test():
    import time
    back_wheels = Back_Wheels()
    DELAY = 0.01
    try:
        back_wheels.forward()
        for i in range(0, 100):
            back_wheels.speed = i
            print("Forward, speed =", i)
            time.sleep(DELAY)
        for i in range(100, 0, -1):
            back_wheels.speed = i
            print("Forward, speed =", i)
            time.sleep(DELAY)

        back_wheels.backward()
        for i in range(0, 100):
            back_wheels.speed = i
            print("Backward, speed =", i)
            time.sleep(DELAY)
        for i in range(100, 0, -1):
            back_wheels.speed = i
            print("Backward, speed =", i)
            time.sleep(DELAY)
    except KeyboardInterrupt:
        print("KeyboardInterrupt, motor stop")
        back_wheels.stop()
    finally:
        print("Finished, motor stop")
        back_wheels.stop()
def getTurningRadiusRatio(angle):
    wheelToWheelLength = 13.80
    wheelToWheelWidth = 13.7 - 2.6
    if angle == 90:
        return 1, False
    else:
        backward = False
        TurningRadius = wheelToWheelLength / np.tan(np.deg2rad(90-angle))
        radiusWheelLeft = TurningRadius - wheelToWheelWidth/2
        radiusWheelRigth = TurningRadius + wheelToWheelWidth/2
        speedRatio =  (radiusWheelLeft/radiusWheelRigth) 
        if angle < 50 or angle > 130:
          backward = True
        return speedRatio, backward

if __name__ == '__main__':
    for angle in [40,45,55,65,75,85,90,95,105,115,125,135,145]:
        print("Angle " + str(angle) + ", ratio: " + str(getTurningRadiusRatio(np.deg2rad(angle))))

