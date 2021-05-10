import time
import math

import RPi.GPIO as GPIO
from simple_pid import PID

from movement import drive
from sensing import mpu6050

# IMPORTANT VARIABLES TO CONFIGURE -------------------

setpoint = 7

Kp = 1
Ki = 0
Kd = 0

ANGLE_OFFSET = 0

# IMPORTANT VARIABLES TO CONFIGURE -------------------

GPIO.setmode(GPIO.BCM)

# STABILITY SWITCH
STABILITY_SWITCH_PIN = 16
GPIO.setup(STABILITY_SWITCH_PIN, GPIO.IN)

# Stepper activation pin
STEPPER_ACTIVATOR_PIN = 24
GPIO.setup(STEPPER_ACTIVATOR_PIN, GPIO.OUT)
GPIO.output(STEPPER_ACTIVATOR_PIN, GPIO.input(STABILITY_SWITCH_PIN))

mpu = mpu6050.mpu6050()


def stability_switch_changed(channel):
    GPIO.output(STEPPER_ACTIVATOR_PIN, GPIO.input(STABILITY_SWITCH_PIN))
    print(GPIO.input(STABILITY_SWITCH_PIN))


if __name__ == '__main__':
    # Wait for the input to go low, run the function when it does
    # GPIO.add_event_detect(STABILITY_SWITCH_PIN, GPIO.BOTH,
    #                     callback=stability_switch_changed, bouncetime=200)

    drive = drive.drive(
        left_direction_pin=12,
        left_step_pin=25,
        right_direction_pin=21,
        right_step_pin=20,
        activator_pin=STEPPER_ACTIVATOR_PIN,
        steps_per_revolution=200)

    drive.activate_stepper()

    pid = PID(Kp, Ki, Kd, setpoint=setpoint, output_limits=(0, 100))
    old_time = time.time()

    # init & and start steppers
    drive.turn_both_steppers()

    # cycle used for activation switch checks
    cycle = 0
    try:
        while(True):
            angle_info = mpu.get_angle()
            v = angle_info[0] - ANGLE_OFFSET
            control = int(pid(v))
            control = abs(control)
            print('V:', v, '| control:', control, '| Frequency:',
                  angle_info[3], '| PID weights:', pid.components)

            # switch activated code
            if cycle % 100 == 0:
                cycle = 0
                if GPIO.input(STABILITY_SWITCH_PIN) == 0:
                    drive.deactivate_stepper()
                    time.sleep(0.2)
                    continue
                drive.activate_stepper()
            cycle += 1
            # switch activated code

            if v > setpoint:
                drive.set_stepper_rotation_clockwise(True)
            else:
                drive.set_stepper_rotation_clockwise(False)

            # drive.change_speed_all(control)

    except KeyboardInterrupt:
        drive.deactivate_stepper()
        print('Stopped!')
