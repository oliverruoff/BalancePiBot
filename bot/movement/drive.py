from time import sleep
import RPi.GPIO as GPIO

GPIO.setmode(GPIO.BCM)

CW = 1     # Clockwise Rotation
CCW = 0    # Counterclockwise Rotation
# Steps per Revolution (360 / 1.8) (1,8° per step (oruoff))


def __init__(self,
             left_direction_pin,
             left_step_pin,
             activator_pin,
             right_direction_pin,
             right_step_spin,
             steps_per_revolution=200,
             delay=.0005):
    self.left_direction_pin = left_direction_pin
    self.left_step_pin = left_step_pin
    self.right_direction_pin = right_direction_pin
    self.right_step_pin = right_step_pin
    self.activator_pin = activator_pin
    self.steps_per_revolution = steps_per_revolution
    self.delay = delay
    GPIO.setup(direction_pin, GPIO.OUT)
    GPIO.setup(step_pin, GPIO.OUT)


def activate_stepper(self):
    GPIO.output(self.activator_pin, GPIO.HIGH)


def turn_stepper(self, steps, clockwise=True):
    direction = CCW
    if clockwise:
        direction = CW
    GPIO.output(self.left_direction_pin, direction)
    GPIO.output(self.right_direction_pin, direction)
    for _ in range(steps):
        GPIO.output(self.left_step_pin, GPIO.HIGH)
        GPIO.output(self.right_step_pin, GPIO.HIGH)
        sleep(self.delay)
        GPIO.output(self.left_step_pin, GPIO.LOW)
        GPIO.output(self.right_step_pin, GPIO.LOW)
        sleep(self.delay)


def turn_stepper_degree(self, degree, clockwise=True):
    direction = CCW
    if clockwise:
        direction = CW
    GPIO.output(self.left_direction_pin, direction)
    GPIO.output(self.right_direction_pin, direction)
    for _ in range(int(self.steps_per_revolution/360*degree)):
        GPIO.output(self.left_step_pin, GPIO.HIGH)
        GPIO.output(self.right_step_pin, GPIO.HIGH)
        sleep(self.delay)
        GPIO.output(self.left_step_pin, GPIO.LOW)
        GPIO.output(self.right_step_pin, GPIO.LOW)
        sleep(self.delay)
