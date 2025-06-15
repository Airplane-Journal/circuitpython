"""
First try at a class for the TT motors
"""

import time
import busio
import board
import supervisor
import adafruit_vl53l1x
import adafruit_hcsr04
from rainbowio import colorwheel
from random import randint
from adafruit_seesaw import neopixel, seesaw
from adafruit_seesaw.pwmout import PWMOut
from adafruit_motor import motor

supervisor.runtime.autoreload = False

i2c = busio.I2C(board.SCL, board.SDA, frequency=400_000)  # default freq. 100 kHz

ss = seesaw.Seesaw(i2c)

class DriveSystem:
    # should this be a child class of motor?
    # class attributes (shouldn't be changed in code?)
    PWM_FREQ = 25  # sustom PWM frequency; Crickit min/max 3Hz/720Hz, default is 50Hz
    DECAY_MODE = motor.SLOW_DECAY  # set controller to Slow Decay (braking) mode
    PWM_PIN_A = 23
    PWM_PIN_B = 22
    PWM_PIN_C = 18
    PWM_PIN_D = 19

    # constructor
    def __init__(self, seesaw):
        # instance attributes (I admit my confusion)
        self.ss = seesaw  # is it ok to use ss?

        self.pwm_a = PWMOut(self.ss, PWM_PIN_A)
        self.pwm_b = PWMOut(self.ss, PWM_PIN_B)
        self.port_motor = motor.DCMotor(self.pwm_a, self.pwm_b)
        self.port_motor.decay_mode = DECAY_MODE

        self.pwm_c = PWMOut(self.ss, PWM_PIN_C)
        self.pwm_d = PWMOut(self.ss, PWM_PIN_D)
        self.stbd_motor = motor.DCMotor(self.pwm_c, self.pwm_d)
        self.stbd_motor.decay_mode = DECAY_MODE

        self.ss.set_pwm_freq(PWM_PIN_A, PWM_FREQ)  # set pins to custom PWM frequency
        self.ss.set_pwm_freq(PWM_PIN_B, PWM_FREQ)
        self.ss.set_pwm_freq(PWM_PIN_C, PWM_FREQ)
        self.ss.set_pwm_freq(PWM_PIN_D, PWM_FREQ)
        print("PWM frequency:", PWM_FREQ)  # display internal PWM frequency

        self.port_motor.throttle = 0  # stop motors
        self.stbd_motor.throttle = 0

    # methods
    def stop(self):
        self.port_motor.throttle = 0
        self.stbd_motor.throttle = 0

    def coast(self):
        self.port_motor.throttle = None  # will this work in Slow Decay Mode?
        self.stbd_motor.throttle = None

    def forward(self, throttle_speed=0.6):
        self.port_motor.throttle = throttle_speed
        self.stbd_motor.throttle = throttle_speed

    def reverse(self, throttle_speed=0.6):
        self.port_motor.throttle = -throttle_speed
        self.stbd_motor.throttle = -throttle_speed

    def rotate(self, direction):
        # keep turning simple for now
        turn_speed = 0.5
        if direction is port:
            self.port_motor.throttle = turn_speed
            self.stbd_motor.throttle = None
        if direction is stbd:
            self.port_motor.throttle = None
            self.stbd_motor.throttle = turn_speed

""" okay try to create a DriveSystem object """

robot_drive = DriveSystem(ss)

robot_drive.stop()
time.sleep(1.0)

robot_drive.forward()
time.sleep(2.0)

robot_drive.stop()
time.sleep(1.0)

robot_drive.reverse()
time.sleep(2.0)

robot_drive.coast()
time.sleep(1.0)

robot_drive.rotate(port)
time.sleep(2.0)

robot_drive.coast()
time.sleep(1.0)

robot_drive.rotate(stbd)
time.sleep(2.0)

robot_drive.stop()
time.sleep(1.0)

print("did it work?")