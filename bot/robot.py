import time

import RPi.GPIO as GPIO

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

SPEED_MULTIPLIER = 40
hold_z = 0

last_loop_time = time.time()
try:
   while(True):
        this_time = time.time()
        elapsed_time = this_time - last_loop_time
        last_loop_time = this_time
        z = mpu.get_accel_data()['z']
        speed = int(abs(z * SPEED_MULTIPLIER))
        speed = speed if speed < 100 else 100
        print(time.time(), 'Z Axis:', z, '| Speed adjustment:', speed, '| Running on %i Hz.' %(int(1/elapsed_time)))
        pt.change_speed_all(speed)
        if z > hold_z:
           pt.move_front()
        else:
            pt.move_back()
except KeyboardInterrupt:
    print('Stopped!')

# mpu.get_accel_data()['z']