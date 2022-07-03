from distutils.log import debug
import time

import RPi.GPIO as GPIO
from simple_pid import PID

from sensors import mpu6050
from actuators import l298n

# IMPORTANT VARIABLES TO CONFIGURE -------------------

DEBUG = True
if DEBUG:
    DEBUG_FREQUENCY_ARRAY_SIZE = 2000 # Number of entries for frequency average calculations etc.
    debug_frequency = [] # Last DEBUG_FREQUENCY_ARRAY_SIZE frequencies stored for average calc.


# If robot center weight is not centered
setpoint = 1

# If motors need some minimal duty cycle to spin
MIN_DUTY_CYCLE = 0

# For PID controller
Kp = 25  # 45
Ki = 100  # 0
Kd = 0.2  # 0.05

# IMPORTANT VARIABLES TO CONFIGURE -------------------

GPIO_MODE = GPIO.BCM
GPIO.setmode(GPIO_MODE)


if __name__ == '__main__':

    settings = {
        'Kp': Kp,
        'Ki': Ki,
        'Kd': Kd
    }

    motor_driver = l298n.l298n(
        in1_pin=19,
        in2_pin=13,
        in3_pin=6,
        in4_pin=5,
        ena_pin=26,
        enb_pin=11,
        gpio_mode=GPIO_MODE)

    pid = PID(settings['Kp'], settings['Ki'], settings['Kd'], setpoint=setpoint,
              sample_time=0.005, output_limits=(-100, 100))

    mpu = mpu6050.mpu6050()

    try:
        while(True):
            now = time.time()

            # Get angle from mpu sensor
            data = mpu.get_angle()
            comp_angle = int(data[0])
            gyro_angle = int(data[1])
            accel_angle = int(data[2])
            frequency = int(data[3])

            # Use pid to get motor control
            control = pid(comp_angle)

            # Increase control in case it's lower than MIN_DUTY_CYCLE
            abs_control = abs(control)
            abs_min_control = MIN_DUTY_CYCLE if abs_control < MIN_DUTY_CYCLE else abs_control

            # Code for debugging. (Sensor and PID output etc.)
            if DEBUG:
                debug_frequency.append(frequency)
                if len(debug_frequency) > DEBUG_FREQUENCY_ARRAY_SIZE:
                    debug_frequency.pop(0)
                freq_avg = sum(debug_frequency) / len(debug_frequency)
                freq_min = min(debug_frequency)
                freq_max = max(debug_frequency)
                print('compl:', comp_angle, '\tgyro:', gyro_angle, '\taccel:', accel_angle, '\tcontrol:', abs_min_control,
                 '\tfreq:', frequency, '\tavg_freq:', freq_avg, '\tmin_freq:', freq_min, '\tmax_freq:', freq_max)

            # if robot falls over, do nothing
            if abs(comp_angle) > 30:
                motor_driver.stop_both()
                continue

            # setting direction
            if control > setpoint:
                motor_driver.change_left_direction(True)
                motor_driver.change_right_direction(True)
            else:
                motor_driver.change_left_direction(False)
                motor_driver.change_right_direction(False)

            # Change motor speed
            motor_driver.change_left_duty_cycle(
                abs_min_control)
            motor_driver.change_right_duty_cycle(
                abs_min_control)

    except KeyboardInterrupt:
        motor_driver.stop_both()
        GPIO.cleanup()
        print('Stopped!')
