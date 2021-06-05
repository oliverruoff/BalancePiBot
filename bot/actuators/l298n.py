# Motor driver control for driver l298n
# author: Oliver Ruoff


import RPi.GPIO as GPIO

GPIO.setmode(GPIO.BCM)


class l298n:

    def __init__(
        self,
        in1_pin,
        in2_pin,
        in3_pin,
        in4_pin,
        ena_pin,
        enb_pin,
        gpio_mode=GPIO.BCM
    ):

        self.in1_pin = in1_pin
        self.in2_pin = in2_pin
        self.in3_pin = in3_pin
        self.in4_pin = in4_pin
        self.left_duty_cycle = 0
        self.right_duty_cycle = 0
        self.left_motor_direction = True
        self.right_motor_direction = True
        self.left_motor_factor = 1
        self.left_motor_factor = 1

        # initializing pins
        GPIO.setmode(gpio_mode)
        GPIO.setup(in1_pin, GPIO.OUT)
        GPIO.setup(in2_pin, GPIO.OUT)
        GPIO.setup(in3_pin, GPIO.OUT)
        GPIO.setup(in4_pin, GPIO.OUT)
        GPIO.setup(ena_pin, GPIO.OUT)
        GPIO.setup(enb_pin, GPIO.OUT)

        # setting all pins to low at start
        self.change_left_direction(True)
        self.change_right_direction(True)

        # right motor
        p_a = GPIO.PWM(enb_pin, 1000)
        self.p_a = p_a
        p_a.start(0)

        # left motor
        p_b = GPIO.PWM(ena_pin, 1000)
        self.p_b = p_b
        p_b.start(0)

        p_a.ChangeDutyCycle(self.left_duty_cycle)
        p_b.ChangeDutyCycle(self.right_duty_cycle)

    def change_left_duty_cycle(self, duty_cycle):
        if duty_cycle > 100:
            duty_cycle = 100
        elif duty_cycle < 0:
            duty_cycle = 0
        self.left_duty_cycle = duty_cycle
        self.p_a.ChangeDutyCycle(duty_cycle)

    def change_right_duty_cycle(self, duty_cycle):
        if duty_cycle > 100:
            duty_cycle = 100
        elif duty_cycle < 0:
            duty_cycle = 0
        self.right_duty_cycle = duty_cycle
        self.p_b.ChangeDutyCycle(duty_cycle)

    def change_right_direction(self, clockwise):
        if clockwise:
            GPIO.output(self.in1_pin, GPIO.LOW)
            GPIO.output(self.in2_pin, GPIO.HIGH)
        else:
            GPIO.output(self.in1_pin, GPIO.HIGH)
            GPIO.output(self.in2_pin, GPIO.LOW)

    def change_left_direction(self, clockwise):
        if clockwise:
            GPIO.output(self.in3_pin, GPIO.LOW)
            GPIO.output(self.in4_pin, GPIO.HIGH)
        else:
            GPIO.output(self.in3_pin, GPIO.HIGH)
            GPIO.output(self.in4_pin, GPIO.LOW)

    def stop_left(self):
        GPIO.output(self.in1_pin, GPIO.LOW)
        GPIO.output(self.in2_pin, GPIO.LOW)

    def stop_right(self):
        GPIO.output(self.in3_pin, GPIO.LOW)
        GPIO.output(self.in4_pin, GPIO.LOW)

    def stop_both(self):
        self.stop_left()
        self.stop_right()
