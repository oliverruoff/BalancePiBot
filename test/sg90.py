import RPi.GPIO as GPIO
from time import sleep

GPIO_MODE = GPIO.BCM

DATA_PIN = 21

GPIO.setmode(GPIO_MODE)
GPIO.setup(DATA_PIN, GPIO.OUT)

pwm=GPIO.PWM(DATA_PIN, 50)
pwm.start(0)

def set_angle(angle):
    duty = angle / 18 + 3
    GPIO.output(DATA_PIN, True)
    pwm.ChangeDutyCycle(duty)
    sleep(1)
    GPIO.output(DATA_PIN, False)
    pwm.ChangeDutyCycle(duty)


print('0')
set_angle(0)
sleep(1)

print('90')
set_angle(90)
sleep(1)

print('180')
set_angle(180)
sleep(1)

print('360')
set_angle(360)