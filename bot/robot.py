import time
import math

import RPi.GPIO as GPIO
from simple_pid import PID

from movement import powertrain
from sensing import mpu6050

GPIO.setmode(GPIO.BCM)

# powertrain
POWERTRAIN_IN1_PIN = 19
POWERTRAIN_IN2_PIN = 13
POWERTRAIN_IN3_PIN = 6
POWERTRAIN_IN4_PIN = 5
POWERTRAIN_ENA_PIN = 26
POWERTRAIN_ENB_PIN = 11
MOTORSPEED_LEFT = 75
MOTORSPEED_RIGHT = 75


pt = powertrain.powertrain(
    POWERTRAIN_IN1_PIN,
    POWERTRAIN_IN2_PIN,
    POWERTRAIN_IN3_PIN,
    POWERTRAIN_IN4_PIN,
    POWERTRAIN_ENA_PIN,
    POWERTRAIN_ENB_PIN,
    MOTORSPEED_LEFT,
    MOTORSPEED_RIGHT)
    
mpu = mpu6050.mpu6050()

pt.change_speed_all(100)

###################### Testing
#while True:
#    i = int(input('speed?\n'))
#    pt.change_speed_all(i)

###################### Testing
###################### PID

setpoint = -1
min_motor_speed = 0 # required for motors to start turning (normally around 55)

Kp = 8
Ki = 0
Kd = 0

pid = PID(Kp, Ki, Kd, setpoint=setpoint, sample_time=0.008, output_limits=(-100, 100))
old_time = time.time()

try:
   while(True):
        angle_info = mpu.get_angle()
        v = angle_info[0]
        control = int(pid(v))
        if v > setpoint:
            pt.move_front()
        else:
            pt.move_back()
        control = abs(control)
        control = min_motor_speed if control < min_motor_speed else control
        pt.change_speed_all(control)
        print('V:', v, '| control:', control, '| Frequency:', angle_info[3], '| PID wights:', pid.components)

except KeyboardInterrupt:
    print('Stopped!')

# mpu.get_accel_data()['z']