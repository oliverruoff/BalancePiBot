import time

import RPi.GPIO as GPIO
from simple_pid import PID

from actuators import l298n
from sensors import mpu6050

# IMPORTANT VARIABLES TO CONFIGURE -------------------

setpoint = 0

Kp = 1
Ki = 0
Kd = 0

# IMPORTANT VARIABLES TO CONFIGURE -------------------
GPIO_MODE = GPIO.BCM
GPIO.setmode(GPIO_MODE)

# STABILITY SWITCH
STABILITY_SWITCH_PIN = 16
GPIO.setup(STABILITY_SWITCH_PIN, GPIO.IN)

mpu = mpu6050.mpu6050()


def stability_switch_changed(channel):
    GPIO.output(STEPPER_ACTIVATOR_PIN, GPIO.input(STABILITY_SWITCH_PIN))
    print(GPIO.input(STABILITY_SWITCH_PIN))


if __name__ == '__main__':
    # Wait for the input to go low, run the function when it does
    # GPIO.add_event_detect(STABILITY_SWITCH_PIN, GPIO.BOTH,
    #                     callback=stability_switch_changed, bouncetime=200)

    motor_driver = l298n.l298n(
        in1_pin=19,
        in2_pin=13,
        in3_pin=6,
        in4_pin=5,
        ena_pin=26,
        enb_pin=11,
        gpio_mode=GPIO_MODE)

    pid = PID(Kp, Ki, Kd, setpoint=setpoint, output_limits=(-100, 100))
    old_time = time.time()

    try:
        while(True):
            angle_info = mpu.get_angle()
            v = angle_info[0]
            control = int(pid(v))
            control = abs(control)
            print('V:', v, '| control:', control, '| Frequency:',
                  angle_info[3], '| PID weights:', pid.components)

            # setting direction
            if control > setpoint:
                motor_driver.change_left_direction(True)
                motor_driver.change_right_direction(True)
            else:
                motor_driver.change_left_direction(False)
                motor_driver.change_right_direction(False)
            # setting motor speed
            motor_driver.change_right_duty_cycle(control)
            motor_driver.change_left_duty_cycle(control)

    except KeyboardInterrupt:
        motor_driver.stop_both()
        print('Stopped!')
