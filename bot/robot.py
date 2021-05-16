import time

import RPi.GPIO as GPIO
from simple_pid import PID

from actuators import l298n
from sensors import mpu6050

# IMPORTANT VARIABLES TO CONFIGURE -------------------

# If robot center weight is not centered
SETPOINT = 0

# If motors need some specify duty cycle to spin
MIN_DUTY_CYCLE = 10

# For PID controller
Kp = 6
Ki = 0
Kd = 0

# IMPORTANT VARIABLES TO CONFIGURE -------------------
GPIO_MODE = GPIO.BCM
GPIO.setmode(GPIO_MODE)

# STABILITY SWITCH
STABILITY_SWITCH_PIN = 17
GPIO.setup(STABILITY_SWITCH_PIN, GPIO.IN)

mpu = mpu6050.mpu6050()

if __name__ == '__main__':

    motor_driver = l298n.l298n(
        in1_pin=19,
        in2_pin=13,
        in3_pin=6,
        in4_pin=5,
        ena_pin=26,
        enb_pin=11,
        gpio_mode=GPIO_MODE)

    pid = PID(Kp, Ki, Kd, setpoint=SETPOINT,
              sample_time=0.005, output_limits=(-100, 100))
    old_time = time.time()

    try:
        while(True):
            stability_switch = GPIO.input(STABILITY_SWITCH_PIN)
            if not stability_switch:
                motor_driver.stop_both()
                time.sleep(0.1)

            angle_info = mpu.get_angle()
            v = angle_info[0]
            control = int(pid(v))

            # setting direction
            if control > SETPOINT:
                motor_driver.change_left_direction(True)
                motor_driver.change_right_direction(True)
            else:
                motor_driver.change_left_direction(False)
                motor_driver.change_right_direction(False)
            # setting motor speed
            control = abs(control)
            control = MIN_DUTY_CYCLE if control < MIN_DUTY_CYCLE else control

            print('V:', v, '| control:', control, '| Frequency:',
                  angle_info[3], '| PID weights:', pid.components)

            motor_driver.change_right_duty_cycle(abs(control))
            motor_driver.change_left_duty_cycle(abs(control))

    except KeyboardInterrupt:
        motor_driver.stop_both()
        print('Stopped!')
