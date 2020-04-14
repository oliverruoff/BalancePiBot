import smbus

from sensing.Kalman import KalmanAngle

class GyroAccel():

    def __init__(self):
        self.kalmanX = KalmanAngle()
        self.kalmanY = KalmanAngle()

        self.RestrictPitch = False	#Comment out to restrict roll to ±90deg instead - please read: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf
        self.radToDeg = 57.2957786
        self.kalAngleX = 0
        self.kalAngleY = 0
        #some MPU6050 Registers and their Address
        self.PWR_MGMT_1   = 0x6B
        self.SMPLRT_DIV   = 0x19
        self.CONFIG       = 0x1A
        self.GYRO_CONFIG  = 0x1B
        self.INT_ENABLE   = 0x38
        self.ACCEL_XOUT_H = 0x3B
        self.ACCEL_YOUT_H = 0x3D
        self.ACCEL_ZOUT_H = 0x3F
        self.GYRO_XOUT_H  = 0x43
        self.GYRO_YOUT_H  = 0x45
        self.GYRO_ZOUT_H  = 0x47
        self.bus = smbus.SMBus(1) 	# or bus = smbus.SMBus(0) for older version boards
        self.DeviceAddress = 0x68   # MPU6050 device address