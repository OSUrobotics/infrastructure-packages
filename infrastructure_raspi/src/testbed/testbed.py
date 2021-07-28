#!/usr/bin/env python

#from os import DirEntry
from time import time
import spidev
import RPi.GPIO as gpio
from stepper_motor import StepperMotor

class Testbed():  # this is a test

    def __init__(self):
        
        
        # Variables for moving cone up and down
        self.reset_cone_pul = 19 # pin
        self.reset_cone_dir = 20  # pin
        self.reset_cone_en = 12  # pin  (HIGH to Enable / LOW to Disable)

        self.reset_cone_speed = 0.000001 # default value

        #TODO
        self.cone_limit_switch = 10  # pin
        
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

        #TODO
        # hall effect sensor for rotating table
        self.hall_effect  = 100  # pin

        # Variables for turntable/encoder wheel
        self.turntable_motor_pwm = 4 # pin 7
        self.turntable_motor_in1 = 21 # pin 
        self.turntable_motor_in2 = 27 # pin 
        self.turntable_motor_en = 13

        # Variables for comunication with arduino for turntable encoder

        # Setting up the pins
        gpio.setwarnings(False)
        gpio.setmode(gpio.BCM)
        gpio.setup(self.reset_cone_pul, gpio.OUT)
        gpio.setup(self.reset_cone_dir, gpio.OUT)
        gpio.setup(self.reset_cone_en, gpio.OUT)

        gpio.setup(self.reset_cable_pul, gpio.OUT)
        gpio.setup(self.reset_cable_dir, gpio.OUT)
        gpio.setup(self.reset_cable_en, gpio.OUT)

        # setting up turntable pins
        gpio.setup(self.turntable_motor_pwm, gpio.OUT)
        gpio.setup(self.turntable_motor_in1, gpio.OUT)
        gpio.setup(self.turntable_motor_in2, gpio.OUT)
        gpio.setup(self.turntable_motor_en, gpio.OUT)

        gpio.output(self.turntable_motor_en, gpio.HIGH)
        gpio.output(self.turntable_motor_in1, gpio.LOW)
        gpio.output(self.turntable_motor_in2, gpio.LOW)
        self.turntable_pwm = gpio.PWM(self.turntable_motor_pwm, 1000)
        self.turntable_pwm.start(50)

        gpio.setup(self.cone_limit_switch, gpio.IN, pull_up_down = gpio.PUD_DOWN)
        gpio.setup(self.hall_effect, gpio.IN, pull_up_down = gpio.PUD_DOWN)
        gpio.setup(self.cone_button, gpio.IN, pull_up_down = gpio.PUD_DOWN)

        # sets up the stepper motors
        self.reset_cone_motor = StepperMotor(self.reset_cone_pul, self.reset_cone_dir, self.reset_cone_en, self.reset_cone_speed)
        self.reset_cable_motor = StepperMotor(self.reset_cable_pul, self.reset_cable_dir, self.reset_cable_en, self.reset_cable_speed)



    def testbed_reset(self, angle=None):

        self.cone_reset_up()
        self.cable_reset_spool_in()
        self.cone_reset_down()
        self.cable_reset_spool_out()


    def cone_reset_up(self, time_duration=None):
        if time_duration == None:
            time_duration = self.lift_time_limit
        start_time = time()
        lift_time = 0
        while True:
            if lift_time >= self.lift_time_limit or gpio.input(self.cone_limit_switch) == gpio.HIGH:
                break
            self.reset_cone_motor.move_for(0.001, self.reset_cone_motor.CCW)
            lift_time = time() - start_time

    def cone_reset_down(self, time_duration=None):  # look at switching to steps moved
        if time_duration == None:
            time_duration = self.lower_time_limit
        start_time = time()
        lower_time = 0
        while True:
            if lower_time >= time_duration:
                break
            self.reset_cone_motor.move_for(0.1, self.reset_cone_motor.CW)
            lower_time = time() - start_time

#    def reset_turntable(self):
#        while True:
#            if 
#        pass

    def cable_reset_spool_in(self):
        start_time = time()
        spool_in_time = 0
        while True:
            pin_value = gpio.input(self.cone_button)
            if spool_in_time >= self.spool_in_time_limit or pin_value == gpio.HIGH:
                if pin_value == gpio.HIGH:
                    print("button was pressed")
                break
                
            self.reset_cable_motor.move_for(0.001, self.reset_cable_motor.CCW)  # check rotations
            spool_in_time = time() - start_time
        

    def cable_reset_spool_out(self):
        start_time = time()
        spool_out_time = 0
        while True:
            if spool_out_time >= self.spool_out_time_limit:
                break
            self.reset_cable_motor.move_for(0.1, self.reset_cable_motor.CW)  # check rotations
            spool_out_time = time() - start_time

    def turntable_reset_home(self):
        delay = 0
        if gpio.input(self.hall_effect) == gpio.HIGH:  # ensures that it goes to the propper home orientation.
            delay = 2
        gpio.output(self.turntable_motor_in1, gpio.LOW)
        gpio.output(self.turntable_motor_in2, gpio.HIGH)
        
        sleep(delay)
        
        while True:
            if gpio.input(self.hall_effect) == gpio.HIGH:
                gpio.output(self.motor_in1, gpio.LOW)
                gpio.output(self.motor_in2, gpio.LOW)
                break
            sleep(0.001)

    def turntable_move_angle(self, goal_angle):

        # tell the arduino to start counting

        gpio.output(self.turntable_motor_in1, gpio.LOW)
        gpio.output(self.turntable_motor_in2, gpio.HIGH)

        while True:
            # get current angle from the arduino

            if current_angle >= goal_angle:
                gpio.output(self.motor_in1, gpio.LOW)
                gpio.output(self.motor_in2, gpio.LOW)
                break

            sleep(0.001)

if __name__ == '__main__':

    test_num = input("""
0) full table reset
1) spool_out
2) spool_in
3) cone_up
4) cone_down
5) turntable_home
6) turntable_angle

What do you want to test? (enter the number)
""")


    reset_testbed = Testbed()
    if test_num == 0:
        reset_testbed.testbed_reset()

    elif test_num == 1:

        reset_testbed.cable_reset_spool_out()

    elif test_num == 2:
        reset_testbed.cable_reset_spool_in()

    elif test_num == 3:
        reset_testbed.cone_reset_up()
        
    elif test_num == 4:
        reset_testbed.cone_reset_down()
    
    elif test_num == 5:
        reset_testbed.turntable_home()
    
    elif test_num == 6:
        angle = input("\nwhat angle do you want to rotate by?  (Degrees)\n")
        reset_testbed.turntable_move_angle()
    else:
        print("\nNot implemented\n")