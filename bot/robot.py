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

pt.move_front()
time.sleep(2)
pt.move_back()
time.sleep(2)
pt.break_right_wheel()
pt.turn_left_wheel(True)
time.sleep(2)
pt.turn_left_wheel(False)
time.sleep(2)
pt.break_left_wheel()
pt.turn_right_wheel(True)
time.sleep(2)
pt.turn_right_wheel(False)
time.sleep(2)
pt.break_motors()
# mpu.get_accel_data()['z']