import time

import RPi.GPIO as GPIO
from simple_pid import PID

from sensors import mpu6050
from actuators import l298n
from inout import inou

# IMPORTANT VARIABLES TO CONFIGURE -------------------

# URL to send telemtry data to
SERVER_URL = '192.168.178.32:5000/telemetry'

DEBUG = True
TELEMTRY_TRANSMISSION = True

# If robot center weight is not centered
SETPOINT = 0

# If motors need some specify duty cycle to spin
MIN_DUTY_CYCLE = 0

# For PID controller
Kp = 14
Ki = 200
Kd = 0.6

# IMPORTANT VARIABLES TO CONFIGURE -------------------
GPIO_MODE = GPIO.BCM
GPIO.setmode(GPIO_MODE)

# STABILITY SWITCH
STABILITY_SWITCH_PIN = 17
GPIO.setup(STABILITY_SWITCH_PIN, GPIO.IN)

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

    mpu = mpu6050.mpu6050()

    old_time = time.time()

    try:
        while(True):
            stability_switch = GPIO.input(STABILITY_SWITCH_PIN)
            if not stability_switch:
                motor_driver.stop_both()
                time.sleep(0.01)
                accel_avg = mpu.get_accel_error()
                mpu.accel_avg = accel_avg
                print('Recalibrated accel error:', accel_avg)

            this_time = time.time()
            frequency = 1 / (this_time - old_time)
            old_time = this_time

            data = mpu.get_angle()

            comp_angle = int(data[0])
            gyro_angle = int(data[1])
            accel_angle = int(data[2])
            frequency = int(data[3])

            control = pid(comp_angle)

            # setting direction
            if control > SETPOINT:
                motor_driver.change_left_direction(True)
                motor_driver.change_right_direction(True)
            else:
                motor_driver.change_left_direction(False)
                motor_driver.change_right_direction(False)
            # setting motor speed
            control = abs(control)
            control = MIN_DUTY_CYCLE if control < MIN_DUTY_CYCLE and control > 0 else control

            if DEBUG:
                print('compl:', comp_angle, 'gyro:', gyro_angle, 'accel:', accel_angle, 'freq:',
                      frequency, 'control:', control)

            # sending telemetry data to server
            if TELEMTRY_TRANSMISSION:
                try:
                    inou.post_telemetry(SERVER_URL, time.time(),
                                        comp_angle, gyro_angle, accel_angle, control, frequency)
                except:
                    print('Couldn`t connect to server..')

            motor_driver.change_right_duty_cycle(abs(control))
            motor_driver.change_left_duty_cycle(abs(control))

    except KeyboardInterrupt:
        motor_driver.stop_both()
        print('Stopped!')
