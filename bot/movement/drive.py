from time import sleep
import RPi.GPIO as GPIO
import pigpio

GPIO.setmode(GPIO.BCM)

STEPPER_DUTY_CYCLE = 64

CW = 1     # Clockwise Rotation
CCW = 0    # Counterclockwise Rotation
# Steps per Revolution (360 / 1.8) (1,8° per step (oruoff))


class drive:

    def __init__(self,
                 left_direction_pin,
                 left_step_pin,
                 right_direction_pin,
                 right_step_pin,
                 activator_pin,
                 steps_per_revolution=200,
                 delay=.0005):
        self.left_direction_pin = left_direction_pin
        self.left_step_pin = left_step_pin
        self.right_direction_pin = right_direction_pin
        self.right_step_pin = right_step_pin
        self.activator_pin = activator_pin
        self.pi = pi = pigpio.pi()
        self.stepper_rotation_clockwise = None
        self.stepper_activated = False
        self.steps_per_revolution = steps_per_revolution
        self.delay = delay

        GPIO.setup(left_direction_pin, GPIO.OUT)
        GPIO.setup(left_step_pin, GPIO.OUT)
        GPIO.setup(right_direction_pin, GPIO.OUT)
        GPIO.setup(right_step_pin, GPIO.OUT)

        pi.set_mode(left_direction_pin, pigpio.OUTPUT)
        pi.set_mode(left_step_pin, pigpio.OUTPUT)
        pi.set_mode(right_direction_pin, pigpio.OUTPUT)
        pi.set_mode(right_step_pin, pigpio.OUTPUT)

    def activate_stepper(self):
        if self.stepper_activated:
            return
        self.stepper_activated = True
        GPIO.output(self.activator_pin, GPIO.HIGH)
        self.pi.set_PWM_dutycycle(self.left_step_pin, STEPPER_DUTY_CYCLE)
        self.pi.set_PWM_dutycycle(self.right_step_pin, STEPPER_DUTY_CYCLE)

    def deactivate_stepper(self):
        if self.stepper_activated:
            self.stepper_activated = False
            GPIO.output(self.activator_pin, GPIO.LOW)
            self.pi.set_PWM_dutycycle(self.left_step_pin, 0)
            self.pi.set_PWM_dutycycle(self.right_step_pin, 0)

    def change_speed_all(self, frequency):
        self.pi.set_PWM_frequency(self.left_step_pin, frequency)
        self.pi.set_PWM_frequency(self.right_step_pin, frequency)

    def set_stepper_rotation_clockwise(self, clockwise):
        if self.stepper_rotation_clockwise == clockwise:
            return
        self.stepper_rotation_clockwise = clockwise
        if clockwise:
            GPIO.output(self.left_direction_pin, CW)
            GPIO.output(self.right_direction_pin, CW)
        else:
            GPIO.output(self.left_direction_pin, CCW)
            GPIO.output(self.right_direction_pin, CCW)

    def turn_both_steppers(self, frequency=1000, clockwise=True):
        self.set_stepper_rotation_clockwise(clockwise)
        self.pi.set_PWM_dutycycle(
            self.left_step_pin, STEPPER_DUTY_CYCLE)  # PWM 1/2 On 1/2 Off
        self.pi.set_PWM_dutycycle(self.right_step_pin, STEPPER_DUTY_CYCLE)
        # 320 / 400 / 500 / 800 / 1000 -> frequency
        self.pi.set_PWM_frequency(self.left_step_pin, frequency)
        self.pi.set_PWM_frequency(self.right_step_pin, frequency)

    def turn_stepper_degree(self, degree, clockwise=True, delay=self.delay):
        direction = CCW
        if clockwise:
            direction = CW
        GPIO.output(self.left_direction_pin, direction)
        GPIO.output(self.right_direction_pin, direction)
        for _ in range(int(self.steps_per_revolution/360*degree)):
            GPIO.output(self.left_step_pin, GPIO.HIGH)
            GPIO.output(self.right_step_pin, GPIO.HIGH)
            sleep(delay)
            GPIO.output(self.left_step_pin, GPIO.LOW)
            GPIO.output(self.right_step_pin, GPIO.LOW)
            sleep(delay)
