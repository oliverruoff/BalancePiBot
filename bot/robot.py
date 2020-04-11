import time

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
    
mpu = mpu6050.mpu6050(0x68)

pt.change_speed_left(100)
pt.change_speed_right(100)

setpoint = 0

Kp = 20
Ki = 1
Kd = 0

pid = PID(Kp, Ki, Kd, setpoint=setpoint, sample_time=0.007, output_limits=(-100, 100))
v = mpu.get_accel_data()['z']

try:
   while(True):
        control = int(pid(v))
        if control < 0:
            pt.move_front()
        else:
            pt.move_back()
        control = abs(control)
        pt.change_speed_all(control)
        print('V:', v, '| control:', control)
        v = mpu.get_accel_data()['z']

except KeyboardInterrupt:
    print('Stopped!')

# mpu.get_accel_data()['z']