'''
        Read Gyro and Accelerometer by Interfacing Raspberry Pi with MPU6050 using Python
        http://www.electronicwings.com
'''
import smbus                    #import SMBus module of I2C
from time import sleep          #import
import time

#some MPU6050 Registers and their Address
PWR_MGMT_1   = 0x6B
SMPLRT_DIV   = 0x19
CONFIG       = 0x1A
GYRO_CONFIG  = 0x1B
INT_ENABLE   = 0x38
ACCEL_XOUT_H = 0x3B
ACCEL_YOUT_H = 0x3D
ACCEL_ZOUT_H = 0x3F
GYRO_XOUT_H  = 0x43
GYRO_YOUT_H  = 0x45
GYRO_ZOUT_H  = 0x47

#Full scale range +/- 250 degree/C as per sensitivity scale factor
MPU_SENSOR_GYRO_CONSTANT = 131.0
MPU_SENSOR_ACCEL_CONSTANT = 16384.0

def MPU_Init():
        #write to sample rate register
        bus.write_byte_data(Device_Address, SMPLRT_DIV, 7)

        #Write to power management register
        bus.write_byte_data(Device_Address, PWR_MGMT_1, 1)

        #Write to Configuration register
        bus.write_byte_data(Device_Address, CONFIG, 0)

        #Write to Gyro configuration register
        bus.write_byte_data(Device_Address, GYRO_CONFIG, 24)

        #Write to interrupt enable register
        bus.write_byte_data(Device_Address, INT_ENABLE, 1)

def read_raw_data(addr):
        #Accelero and Gyro value are 16-bit
        high = bus.read_byte_data(Device_Address, addr)
        low = bus.read_byte_data(Device_Address, addr+1)

        #concatenate higher and lower value
        value = ((high << 8) | low)

        #to get signed value from mpu6050
        if(value > 32768):
                value = value - 65536
        return value

def get_new_gyro_angle(axis, time_diff_s, old_angle=0, gyro_drift = 0.4827480916030538):
    DEGREE_SCALE_CONSTANT = 8
    if axis == 'x':
        raw = read_raw_data(GYRO_XOUT_H)
    elif axis == 'y':
        raw = read_raw_data(GYRO_YOUT_H)
    elif axis == 'z':
        raw = read_raw_data(GYRO_ZOUT_H)
    raw = raw / MPU_SENSOR_GYRO_CONSTANT
    angle = old_angle + ((raw-gyro_drift) * time_diff_s * DEGREE_SCALE_CONSTANT)
    return angle

def get_new_accel_angle(axis):
    if axis == 'x':
        raw = read_raw_data(ACCEL_XOUT_H)
    elif axis == 'y':
        raw = read_raw_data(ACCEL_YOUT_H)
    elif axis == 'z':
        raw = read_raw_data(ACCEL_ZOUT_H)
    raw = raw / MPU_SENSOR_ACCEL_CONSTANT
    return raw

### START

bus = smbus.SMBus(1)    # or bus = smbus.SMBus(0) for older version boards
Device_Address = 0x68   # MPU6050 device address

MPU_Init()

gyro_drift = sum([read_raw_data(GYRO_YOUT_H)/MPU_SENSOR_GYRO_CONSTANT for i in range(100)])/100

angle = 0
last_time = time.time()
while True:
        curr_time = time.time()
        time_diff = curr_time - last_time
        last_time = curr_time
        angle = get_new_gyro_angle('y', time_diff, angle, gyro_drift)
        accel = get_new_accel_angle('y')
        freq = 1 / time_diff
        print('Frequence:', freq, 'Hz | Angle:', angle, '| Accel:', accel)
