import time
import math

import RPi.GPIO as GPIO
from simple_pid import PID

from movement import stepper
from sensing import mpu6050

# IMPORTANT VARIABLES TO CONFIGURE -------------------

setpoint = -1.3
# required for motors to start turning (normally around 55)
min_motor_speed = 40

Kp = 13
Ki = 0
Kd = 1.3

# IMPORTANT VARIABLES TO CONFIGURE -------------------

GPIO.setmode(GPIO.BCM)

# STABILITY SWITCH
STABILITY_SWITCH_PIN = 16
GPIO.setup(STABILITY_SWITCH_PIN, GPIO.IN)

mpu = mpu6050.mpu6050()


def stability_switch_changed(channel):
    print(GIPO.input(STABILITY_SWITCH_PIN))


# Wait for the input to go low, run the function when it does
GPIO.add_event_detect(STABILITY_SWITCH_PIN, GPIO.BOTH,
                      callback=inputLow, bouncetime=200)


def is_stuck(angle, min_angle=-20, max_angle=20):
    if angle > max_angle or angle < min_angle:
        print('Angle: %f. Seems like I fell. Waiting for help.' % (angle))
        speed = pt.get_speed_all()
        if speed[0] > 0 or speed[1] > 0:
            pt.change_speed_all(0)
            pt.break_motors()
        return True
    else:
        return False

# Testing
# while True:
#    i = int(input('speed?\n'))
#    pt.change_speed_all(i)

# Testing
# PID


pid = PID(Kp, Ki, Kd, setpoint=setpoint,
          sample_time=0.008, output_limits=(-100, 100))
old_time = time.time()

try:
    while(True):
        angle_info = mpu.get_angle()
        v = angle_info[0]
        if is_stuck(v):
            continue
        control = int(pid(v))
        if v > setpoint:
            stepper.turn_stepper(1)
        else:
            stepper.turn_stepper(1, false)
        control = abs(control)
        control = min_motor_speed if control < min_motor_speed else control
        pt.change_speed_all(control)
        print('V:', v, '| control:', control, '| Frequency:',
              angle_info[3], '| PID wights:', pid.components)

except KeyboardInterrupt:
    print('Stopped!')

# mpu.get_accel_data()['z']
