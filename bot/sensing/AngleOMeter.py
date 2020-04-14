#Connections
#MPU6050 - Raspberry pi
#VCC - 5V  (2 or 4 Board)
#GND - GND (6 - Board)
#SCL - SCL (5 - Board)
#SDA - SDA (3 - Board)


from sensing.Kalman import KalmanAngle
from sensing.GyroAccel import GyroAccel
import smbus			#import SMBus module of I2C
import time
import math

class AngleOMeter():

	def __init__(self):
		self.gyro_accel = GyroAccel()
		self.MPU_Init()

		time.sleep(1)
        #Read Accelerometer raw value
		self.accX = self.read_raw_data(self.gyro_accel.ACCEL_XOUT_H)
		self.accY = self.read_raw_data(self.gyro_accel.ACCEL_YOUT_H)
		self.accZ = self.read_raw_data(self.gyro_accel.ACCEL_ZOUT_H)

		if (self.gyro_accel.RestrictPitch):
			roll = math.atan2(self.accY, self.accZ) * self.gyro_accel.radToDeg
			pitch = math.atan(-self.accX/math.sqrt((self.accY**2)+(self.accZ**2))) * self.gyro_accel.radToDeg
		else:
			roll = math.atan(self.accY/math.sqrt((self.accX**2)+(self.accZ**2))) * self.gyro_accel.radToDeg
			pitch = math.atan2(-self.accX,self.accZ) * self.gyro_accel.radToDeg
		print(roll)
		self.gyro_accel.kalmanX.setAngle(roll)
		self.gyro_accel.kalmanY.setAngle(pitch)
		self.gyroXAngle = roll
		self.gyroYAngle = pitch
		self.compAngleX = roll
		self.compAngleY = pitch

		self.timer = time.time()
		self.flag = 0


	#Read the gyro and acceleromater values from MPU6050
	def MPU_Init(self):
		#write to sample rate register
		self.gyro_accel.bus.write_byte_data(self.gyro_accel.DeviceAddress, self.gyro_accel.SMPLRT_DIV, 7)

		#Write to power management register
		self.gyro_accel.bus.write_byte_data(self.gyro_accel.DeviceAddress, self.gyro_accel.PWR_MGMT_1, 1)

		#Write to Configuration register
		#Setting DLPF (last three bit of 0X1A to 6 i.e '110' It removes the noise due to vibration.) https://ulrichbuschbaum.wordpress.com/2015/01/18/using-the-mpu6050s-dlpf/
		self.gyro_accel.bus.write_byte_data(self.gyro_accel.DeviceAddress, self.gyro_accel.CONFIG, int('0000110',2))

		#Write to Gyro configuration register
		self.gyro_accel.bus.write_byte_data(self.gyro_accel.DeviceAddress, self.gyro_accel.GYRO_CONFIG, 24)

		#Write to interrupt enable register
		self.gyro_accel.bus.write_byte_data(self.gyro_accel.DeviceAddress, self.gyro_accel.INT_ENABLE, 1)


	def read_raw_data(self, addr):
		#Accelero and Gyro value are 16-bit
			high = self.gyro_accel.bus.read_byte_data(self.gyro_accel.DeviceAddress, addr)
			low = self.gyro_accel.bus.read_byte_data(self.gyro_accel.DeviceAddress, addr+1)

			#concatenate higher and lower value
			value = ((high << 8) | low)

			#to get signed value from mpu6050
			if(value > 32768):
					value = value - 65536
			return value

	def get_final_angles(self):
		if(self.flag > 100): #Problem with the connection
			print("There is a problem with the connection")
			self.flag=0
			return
		try:
			#Read Accelerometer raw value
			self.accX = self.read_raw_data(self.gyro_accel.ACCEL_XOUT_H)
			self.accY = self.read_raw_data(self.gyro_accel.ACCEL_YOUT_H)
			self.accZ = self.read_raw_data(self.gyro_accel.ACCEL_ZOUT_H)

			#Read Gyroscope raw value
			self.gyroX = self.read_raw_data(self.gyro_accel.GYRO_XOUT_H)
			self.gyroY = self.read_raw_data(self.gyro_accel.GYRO_YOUT_H)
			self.gyroZ = self.read_raw_data(self.gyro_accel.GYRO_ZOUT_H)

			dt = time.time() - self.timer
			self.timer = time.time()

			if (self.gyro_accel.RestrictPitch):
				roll = math.atan2(self.accY,self.accZ) * self.gyro_accel.radToDeg
				pitch = math.atan(-self.accX/math.sqrt((self.accY**2)+(self.accZ**2))) * self.gyro_accel.radToDeg
			else:
				roll = math.atan(self.accY/math.sqrt((self.accX**2)+(self.accZ**2))) * self.gyro_accel.radToDeg
				pitch = math.atan2(-self.accX,self.accZ) * self.gyro_accel.radToDeg

			gyroXRate = self.gyroX/131
			gyroYRate = self.gyroY/131

			if (self.gyro_accel.RestrictPitch):

				if((roll < -90 and kalAngleX >90) or (roll > 90 and kalAngleX < -90)):
					self.gyro_accel.kalmanX.setAngle(roll)
					complAngleX = roll
					kalAngleX   = roll
					self.gyroXAngle  = roll
				else:
					kalAngleX = self.gyro_accel.kalmanX.getAngle(roll,gyroXRate,dt)

				if(abs(kalAngleX)>90):
					gyroYRate  = -gyroYRate
					kalAngleY  = self.gyro_accel.kalmanY.getAngle(pitch,gyroYRate,dt)
			else:

				if((pitch < -90 and kalAngleY >90) or (pitch > 90 and kalAngleY < -90)):
					self.gyro_accel.kalmanY.setAngle(pitch)
					complAngleY = pitch
					kalAngleY   = pitch
					self.gyroYAngle  = pitch
				else:
					kalAngleY = self.gyro_accel.kalmanY.getAngle(pitch,gyroYRate,dt)

				if(abs(kalAngleY)>90):
					gyroXRate  = -gyroXRate
					kalAngleX = self.gyro_accel.kalmanX.getAngle(roll,gyroXRate,dt)

			#angle = (rate of change of angle) * change in time
			self.gyroXAngle = gyroXRate * dt
			self.gyroYAngle = self.gyroYAngle * dt

			#compAngle = constant * (old_compAngle + angle_obtained_from_gyro) + constant * angle_obtained from accelerometer
			self.compAngleX = 0.93 * (self.compAngleX + gyroXRate * dt) + 0.07 * roll
			self.compAngleY = 0.93 * (self.compAngleY + gyroYRate * dt) + 0.07 * pitch

			if ((self.gyroXAngle < -180) or (self.gyroXAngle > 180)):
				self.gyroXAngle = kalAngleX
			if ((self.gyroYAngle < -180) or (self.gyroYAngle > 180)):
				self.gyroYAngle = kalAngleY

			print("Angle X: " + str(kalAngleX)+"   " +"Angle Y: " + str(kalAngleY))
			return(kalAngleX, kalAngleY)
			#print(str(roll)+"  "+str(self.gyroXAngle)+"  "+str(self.compAngleX)+"  "+str(kalAngleX)+"  "+str(pitch)+"  "+str(self.gyroYAngle)+"  "+str(self.compAngleY)+"  "+str(kalAngleY))
			time.sleep(0.005)

		except Exception as exc:
			flag += 1
