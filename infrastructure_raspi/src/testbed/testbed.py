#!/usr/bin/env python

#from os import DirEntry
from time import time
#import spidev
import RPi.GPIO as gpio
from stepper_motor import StepperMotor

class Testbed():  # this is a test

    def __init__(self):
        
        
        # Variables for moving cone up and down
        self.reset_cone_pul = 19 # pin
        self.reset_cone_dir = 20  # pin
        self.reset_cone_en = 12  # pin  (HIGH to Enable / LOW to Disable)

        self.reset_cone_speed = 0.000001 # default value

#        self.cone_limit_switch = num  # pin
        
        self.lift_time_limit = 2  # seconds
        self.lower_time_limit = 0.5  # seconds

        #contact plates on the cone - like a button
        self.cone_button = 24  # pin


        # Variables for spooling in/out the cable
        self.reset_cable_pul = 16 # pin Green wire
        self.reset_cable_dir = 6 # pin 31 Red wire
        self.reset_cable_en = 5 # pin 29 Blue wire (HIGH to Enable / LOW to Disable)
        self.reset_cable_speed = 0.000001  # default value

        self.spool_out_time_limit = 0.5  # seconds
        self.spool_in_time_limit = 3  # seconds

        # hall effect sensor for rotating table
        # self.hall_effect  = num  # pin

        # Variables for turntable/encoder wheel
        self.turntable_motor_en = 25  # pin 22
        self.turntable_motor_in1 = 26  # pin  not the correct one
        self.turntable_motor_in2 = 23  # pin 16

        # Variables for comunication with arduino for turntable encoder

        # Setting up the pins
        gpio.setmode(gpio.BCM)
        gpio.setup(self.reset_cone_pul, gpio.OUT)
        gpio.setup(self.reset_cone_dir, gpio.OUT)
        gpio.setup(self.reset_cone_en, gpio.OUT)
        gpio.setup(self.reset_cable_pul, gpio.OUT)
        gpio.setup(self.reset_cable_dir, gpio.OUT)
        gpio.setup(self.reset_cable_en, gpio.OUT)

 #       gpio.setup(self.cone_limit_switch, gpio.IN)
 #       gpio.setup(self.hall_effect, gpio.IN)
        gpio.setup(self.cone_button, gpio.IN, pull_up_down = gpio.PUD_UP)

        self.reset_cone_motor = StepperMotor(self.reset_cone_pul, self.reset_cone_dir, self.reset_cone_en, self.reset_cone_speed)
        self.reset_cable_motor = StepperMotor(self.reset_cable_pul, self.reset_cable_dir, self.reset_cable_en, self.reset_cable_speed)



    def reset(self, angle=None):

        pass


    def cone_reset_up(self):
        start_time = time()
        lift_time = 0
        while True:
            if lift_time >= self.lift_time_limit or gpio.input(self.cone_limit_switch) == True:
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

#    def reset_turntable(self):
#        while True:
#            if 
#        pass

    def cable_reset_spool_in(self):
        start_time = time()
        spool_in_time = 0
        while True:
            if spool_in_time >= self.spool_in_time_limit or gpio.input(self.cone_button) == False:
                break
            self.reset_cable_motor.move_for(0.1, self.reset_cable_motor.CCW)  # check rotations
            spool_in_time = time() - start_time
        if self.cone_button == False:
            print("button was pressed")

    def cable_reset_spool_out(self):
        start_time = time()
        spool_out_time = 0
        while True:
            if spool_out_time >= self.spool_out_time_limit:
                break
            self.reset_cable_motor.move_for(0.1, self.reset_cable_motor.CW)  # check rotations
            spool_out_time = time() - start_time


if __name__ == '__main__':

    test_num = input("""
1) spool_out
2) spool_in
3) cone_up
4) cone_down
5) turntable_home
6) turntable_angle

What do you want to test? (enter the number)
""")


    reset_testbed = Testbed()

    if test_num == 1:

        reset_testbed.cable_reset_spool_out()

    elif test_num == 2:
        reset_testbed.cable_reset_spool_in()

    else:
        print("not implemented yet")
