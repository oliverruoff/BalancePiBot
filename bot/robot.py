import time
import math

import RPi.GPIO as GPIO
from simple_pid import PID

from movement import stepper
from sensing import mpu6050

# IMPORTANT VARIABLES TO CONFIGURE -------------------

setpoint = -86
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

# Complementary filter angle average (1000)
ANGLE_OFFSET = -87.07159316982624

mpu = mpu6050.mpu6050()


def stability_switch_changed(channel):
    print(GIPO.input(STABILITY_SWITCH_PIN))


# Wait for the input to go low, run the function when it does
GPIO.add_event_detect(STABILITY_SWITCH_PIN, GPIO.BOTH,
                      callback=stability_switch_changed, bouncetime=200)


pid = PID(Kp, Ki, Kd, setpoint=setpoint,
          sample_time=0.008, output_limits=(-100, 100))
old_time = time.time()

try:
    while(True):
        angle_info = mpu.get_angle()
        v = angle_info[0] - ANGLE_OFFSET
        control = int(pid(v))
        if v > setpoint:
            stepper.turn_stepper(10)
        else:
            stepper.turn_stepper(10, False)
        control = abs(control)
        control = min_motor_speed if control < min_motor_speed else control
        # pt.change_speed_all(control)
        print('V:', v, '| control:', control, '| Frequency:',
              angle_info[3], '| PID weights:', pid.components)

except KeyboardInterrupt:
    print('Stopped!')

# mpu.get_accel_data()['z']
