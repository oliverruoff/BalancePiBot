import time

import RPi.GPIO as GPIO
from simple_pid import PID

from sensors import mpu6050
from actuators import l298n
from transmission import transmittor

# IMPORTANT VARIABLES TO CONFIGURE -------------------

# URL to send telemtry data to
TELEMTRY_TRANSMISSION = False
TELEMETRY_BATCH_TIME_SECONDS = 1
SERVER_URL = 'http://192.168.178.32:5000'

DEBUG = True


# If robot center weight is not centered
setpoint = 1
# Adds this value to right motor's duty cycle,
# subtracts it from the left motor's duty cycle.
RIGHT_MOTOR_OFFSET = 2

# If motors need some minimal duty cycle to spin
MIN_DUTY_CYCLE = 0

# For PID controller
Kp = 45  # 50
Ki = 0  # 0
Kd = 0.05  # 0.3

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

    if TELEMTRY_TRANSMISSION:
        transmit = transmittor.transmittor(
            SERVER_URL, TELEMETRY_BATCH_TIME_SECONDS)
        transmit.start_config_synchronization()
        print('Synching with telemetry server started.')

    last_telemetry_server_sync = 0

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
            abs_min_control = MIN_DUTY_CYCLE if abs_control < MIN_DUTY_CYCLE and abs_control > 0 else abs_control

            if DEBUG:
                print('compl:', comp_angle, 'gyro:', gyro_angle, 'accel:', accel_angle, 'freq:',
                      frequency, 'control:', abs_min_control)

            # Telemetry server interaction
            if TELEMTRY_TRANSMISSION:
                # Sending telemetry data to server
                transmit.collect_telemetry(
                    comp_angle, gyro_angle, accel_angle, control, frequency)
                # Every second sync with telemetry server
                if (now - last_telemetry_server_sync) >= 1:
                    print('Syncing with telemetry server')
                    new_settings = transmit.get_configs()
                    last_telemetry_server_sync = now
                    if new_settings != settings:
                        print(
                            'Received new settings from telemetry server:', new_settings)
                        settings = new_settings
                        setpoint = settings['Setpoint']
                        pid = PID(settings['Kp'], settings['Ki'], settings['Kd'], setpoint=setpoint,
                                  sample_time=0.005, output_limits=(-100, 100))

            # if robot fell over, do nothing
            if abs(comp_angle) > 30:
                motor_driver.stop_both()
                time.sleep(0.01)
                continue

            # setting direction
            if control > setpoint:
                motor_driver.change_left_direction(True)
                motor_driver.change_right_direction(True)
            else:
                motor_driver.change_left_direction(False)
                motor_driver.change_right_direction(False)

            left_motor_control = abs_min_control - RIGHT_MOTOR_OFFSET if abs_min_control - \
                RIGHT_MOTOR_OFFSET > 0 and abs_min_control - RIGHT_MOTOR_OFFSET < 100 else \
                0 if abs_min_control - RIGHT_MOTOR_OFFSET < 100 else 100

            right_motor_control = abs_min_control + RIGHT_MOTOR_OFFSET if abs_min_control + \
                RIGHT_MOTOR_OFFSET < 100 and abs_min_control + RIGHT_MOTOR_OFFSET > 0 else \
                100 if abs_min_control + RIGHT_MOTOR_OFFSET > 0 else 0

            if DEBUG:
                print('Left motor control:', left_motor_control)
                print('Right motor control:', right_motor_control)

            # Change motor speed
            motor_driver.change_left_duty_cycle(
                left_motor_control)
            motor_driver.change_right_duty_cycle(
                right_motor_control)

    except KeyboardInterrupt:
        motor_driver.stop_both()
        print('Stopped!')
