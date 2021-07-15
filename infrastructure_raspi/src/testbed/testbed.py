#!/usr/bin/env python

from os import DirEntry
from time import time
import spidev
import RPi.GPIO as gpio
from stepper_motor import StepperMotor

class Testbed():

    def __init__(self):
        
        
        # Variables for moving cone up and down
        self.reset_cone_pul = 5  # pin 9
        self.reset_cone_dir = 6  # pin 31
        self.reset_cone_en = 13  # pin 33 (HIGH to Enable / LOW to Disable)

        self.reset_cone_speed = 0.000001 # default value

        self.reset_cable_pul = num  # pin
        self.reset_cable_dir = num  # pin
        self.reset_cable_en = num  # pin  (HIGH to Enable / LOW to Disable)
        self.reset_cable_speed = 0.000001  # default value

        # limit switches
        self.cone_limit_switch = num  # pin

        # hall effect sensor for rotating table
        self.hall_effect  = num  # pin
        
        #contact plates on the cone - like a button
        self.cone_button = num  # pin


        self.spool_out_time_limit = 2  # seconds
        self.spool_in_time_limit = 2  # seconds
        self.lift_time_limit = 2  # seconds
        self.lower_time_limit = 2  # seconds
        

        gpio.setmode(gpio.BCM)
        gpio.setup(self.reset_cone_pul, gpio.OUT)
        gpio.setup(self.reset_cone_dir, gpio.OUT)
        gpio.setup(self.reset_cone_en, gpio.OUT)
        gpio.setup(self.reset_cable_pul, gpio.OUT)
        gpio.setup(self.reset_cable_dir, gpio.OUT)
        gpio.setup(self.reset_cable_en, gpio.OUT)

        gpio.setup(self.cone_limit_switch, gpio.IN)
        gpio.setup(self.hall_effect, gpio.IN)
        gpio.setup(self.cone_button, gpio.IN)

        self.reset_cone_motor = StepperMotor(self.reset_cone_pul, self.reset_cone_dir, self.reset_cone_en, self.reset_cone_speed)
        self.reset_cable_motor = StepperMotor(self.reset_cable_pul, self.reset_cable_dir, self.reset_cable_en, self.reset_cable_speed)



    def reset(self, angle=None):

        pass


    def cone_reset_up(self):
        start_time = time()
        lift_time = 0
        while True:
            if lift_time >= self.lift_time_limit or self.cone_limit_switch == True:
                break
            self.reset_cone_motor.move_for(0.1, self.reset_cone_motor.CCW)
            lift_time = time() - start_time

    def cone_reset_down(self):  # look at switching to steps moved
        start_time = time()
        lower_time = 0
        while True:
            if lower_time >= self.lower_time_limit:
                break
            self.reset_cone_motor.move_for(0.1, self.reset_cone_motor.CW)
            lower_time= time() - start_time

    def table_rotation(self):
        
        pass

    def cable_reset_spool_in(self):
        start_time = time()
        spool_in_time = 0
        while True:
            if spool_in_time >= self.spool_in_time_limit or self.cone_button == True:
                break
            self.reset_cable_motor.move_for(0.1, self.reset_cable_motor.CCW)  # check rotations
            spool_in_time = time() - start_time

    def cable_reset_spool_out(self):
        start_time = time()
        spool_out_time = 0
        while True:
            if spool_out_time >= self.spool_out_time_limit:
                break
            self.reset_cable_motor.move_for(0.1, self.reset_cable_motor.CW)  # check rotations
            spool_out_time = time() - start_time
