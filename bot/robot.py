import time
import math

import RPi.GPIO as GPIO
from simple_pid import PID

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

pt.change_speed_all(100)

###################### Testing

def read_all(address):
    gyro_scale = 131.0
    accel_scale = 16384.0
    raw_gyro_data = bus.read_i2c_block_data(address, 0x43, 6)
    raw_accel_data = bus.read_i2c_block_data(address, 0x3b, 6)

    gyro_scaled_x = twos_compliment((raw_gyro_data[0] << 8) + raw_gyro_data[1]) / gyro_scale
    gyro_scaled_y = twos_compliment((raw_gyro_data[2] << 8) + raw_gyro_data[3]) / gyro_scale
    gyro_scaled_z = twos_compliment((raw_gyro_data[4] << 8) + raw_gyro_data[5]) / gyro_scale

    accel_scaled_x = twos_compliment((raw_accel_data[0] << 8) + raw_accel_data[1]) / accel_scale
    accel_scaled_y = twos_compliment((raw_accel_data[2] << 8) + raw_accel_data[3]) / accel_scale
    accel_scaled_z = twos_compliment((raw_accel_data[4] << 8) + raw_accel_data[5]) / accel_scale

    return (gyro_scaled_x, gyro_scaled_y, gyro_scaled_z, accel_scaled_x, accel_scaled_y, accel_scaled_z)
    
def twos_compliment(val):
    if (val >= 0x8000):
        return -((65535 - val) + 1)
    else:
        return val

def dist(a, b):
    return math.sqrt((a * a) + (b * b))


def get_y_rotation(x,y,z):
    radians = math.atan2(x, dist(y,z))
    return -math.degrees(radians)

def get_x_rotation(x,y,z):
    radians = math.atan2(y, dist(x,z))
    return math.degrees(radians)

#multiplier = 10

while True:
    angle = mpu.get_all_data()
    print('Accel:', angle[0], 'Gyro:', angle[1], 'Temp:', angle[2])
#    s = abs(angle)*multiplier
#    s = 55 if s < 55 else s
#    s = 100 if s > 100 else s
#    pt.change_speed_all(s)

#    if angle > 0:
#        pt.move_front()
#    else:
#        pt.move_back()


###################### Testing
###################### PID

setpoint = -0.38623979442138656
min_motor_speed = 55 # required for motors to start turning

Kp = 30
Ki = 0
Kd = 0

pid = PID(Kp, Ki, Kd, setpoint=setpoint, sample_time=0.007, output_limits=(-100, 100))
v = mpu.get_accel_data()['z']

old_time = time.time()

try:
   while(True):
        control = int(pid(v))
        if control < 0:
            pt.move_front()
        else:
            pt.move_back()
        control = abs(control)
        control = min_motor_speed if control < min_motor_speed else control
        pt.change_speed_all(control)
        current_time = time.time()
        print('V:', v, '| control:', control, '| Time elapsed (s):', current_time-old_time, 'PID wights:', pid.components)
        old_time = current_time
        v = mpu.get_accel_data()['z']

except KeyboardInterrupt:
    print('Stopped!')

# mpu.get_accel_data()['z']