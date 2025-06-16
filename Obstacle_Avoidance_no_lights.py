"""
troubleshoot no lights with motor
"""

import time
import busio
import board
import supervisor
import adafruit_vl53l1x
import adafruit_hcsr04
from adafruit_seesaw import neopixel, seesaw
from adafruit_seesaw.pwmout import PWMOut
from adafruit_motor import motor

supervisor.runtime.autoreload = False

PORT = "port"
STBD = "stbd"
TRIGGER_PIN = board.D5
ECHO_PIN = board.D6
LASER_DISTANCE_MODE = 1
LASER_TIMING_BUDGET = 100
LASER_PAUSE_TIME = 200  # milliseconds x 1000 equals seconds
SONIC_PAUSE_TIME = 400

class DriveSystem:
    """ class to control dc motors together """
    PWM_FREQ = 25  # sustom PWM frequency; Crickit min/max 3Hz/720Hz, default is 50Hz
    DECAY_MODE = motor.SLOW_DECAY  # set controller to Slow Decay (braking) mode
    PWM_PIN_A = 23
    PWM_PIN_B = 22
    PWM_PIN_C = 18
    PWM_PIN_D = 19

    # constructor
    def __init__(self, seesaw):
        self.ss = seesaw

        self.pwm_a = PWMOut(self.ss, self.PWM_PIN_A)
        self.pwm_b = PWMOut(self.ss, self.PWM_PIN_B)
        self.stbd_motor = motor.DCMotor(self.pwm_a, self.pwm_b)
        self.stbd_motor.decay_mode = self.DECAY_MODE

        self.pwm_c = PWMOut(self.ss, self.PWM_PIN_C)
        self.pwm_d = PWMOut(self.ss, self.PWM_PIN_D)
        self.port_motor = motor.DCMotor(self.pwm_c, self.pwm_d)
        self.port_motor.decay_mode = self.DECAY_MODE

        self.ss.set_pwm_freq(self.PWM_PIN_A, self.PWM_FREQ)  # set pins to custom PWM frequency
        self.ss.set_pwm_freq(self.PWM_PIN_B, self.PWM_FREQ)
        self.ss.set_pwm_freq(self.PWM_PIN_C, self.PWM_FREQ)
        self.ss.set_pwm_freq(self.PWM_PIN_D, self.PWM_FREQ)

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

    def rotate(self, direction, turn_speed=0.5):
        if direction == "port":
            self.port_motor.throttle = turn_speed
            self.stbd_motor.throttle = -turn_speed
        if direction == "stbd":
            self.port_motor.throttle = -turn_speed
            self.stbd_motor.throttle = turn_speed

i2c = busio.I2C(board.SCL, board.SDA)

ss = seesaw.Seesaw(i2c)

robot_drive = DriveSystem(ss)

sonar = adafruit_hcsr04.HCSR04(trigger_pin=TRIGGER_PIN, echo_pin=ECHO_PIN)

vl53 = adafruit_vl53l1x.VL53L1X(i2c)
vl53.distance_mode = LASER_DISTANCE_MODE
vl53.timing_budget = LASER_TIMING_BUDGET

# loop variables, constants up top under imports
laser_time = 0
sonic_time = 0
obstacle = False

vl53.start_ranging()

# get your finger out of the way
time.sleep(3)

robot_drive.rotate(STBD, 1)
time.sleep(5)
robot_drive.coast()
time.sleep(2)

while True:

    if obstacle:
        robot_drive.stop()
        print("obstacle!")
        robot_drive.reverse(0.8)
        time.sleep(1)
        robot_drive.stop()
        robot_drive.rotate(PORT, 0.8)
        time.sleep(1)
        obstacle = False
    else:
        robot_drive.forward(0.8)

    if laser_time == 0 or (laser_time + LASER_PAUSE_TIME) < supervisor.ticks_ms():
        if vl53.data_ready:
            laser_distance = vl53.distance
            print(f"Laser Distance: {laser_distance}cm")
            if laser_distance is not None and laser_distance < 18:
                obstacle = True
            vl53.clear_interrupt()
            laser_time = supervisor.ticks_ms()

    if sonic_time == 0 or (sonic_time + SONIC_PAUSE_TIME) < supervisor.ticks_ms():
        try:
            sonic_distance = sonar.distance
            print(f"Sonar Distance: {sonic_distance}cm")
            if sonic_distance < 15:
                obstacle = True
        except RuntimeError:
            pass
        sonic_time = supervisor.ticks_ms()