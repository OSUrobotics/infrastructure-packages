#!/usr/bin/env python


# reset/set node
#
# reset.py
#
# Ryan Roberts
#
# A node that handles the setting/resetting of the hardware

import rospy
import sys
import actionlib
#import hardware_setup
from time import time
import spidev
import RPi.GPIO as gpio
import drawer.VL53L0X as VL53L0X
from drawer.stepper_motor import StepperMotor

#messages
from infrastructure_msgs.msg import StageAction, StageGoal, StageFeedback, StageResult
from infrastructure_msgs.msg import TestParametersAction, TestParametersGoal, TestParametersFeedback, TestParametersResult

class ResetSetController():

    def __init__(self):
        #define action servers
	self.reset_as = actionlib.SimpleActionServer("reset_hardware", StageAction, self.reset_callback, False) 
	self.reset_as.start()
	self.parameters_as = actionlib.SimpleActionServer("set_test_parameters", TestParametersAction, self.parameters_callback, False) 
        self.parameters_as.start()
        #hardware constants/objects
        self.tof = VL53L0X.VL53L0X()
        self.reset_pul = 5 #pin 29
        self.reset_dir = 6 # pin 31 
        self.reset_en = 13  # pin 33, (High to Enable / LOW to Disable)
        self.time_unwind = 2 #in seconds
        self.reset_speed = .000001 # seconds
        self.dis_buffer = 5 #buffer value for resetting drawer (in mm)
        self.fric_pul = 17 #pin 11
        self.fric_dir = 27 # pin 13
        self.fric_en = 22  # pin 15 (High to Enable / LOW to Disable)
        self.fric_steps = .00032 #relation between friction to # of motor steps
        self.fric_speed = .000001 #seconds
        self.fric_min_steps = 2500 #min steps it takes to get brake to touch drawer
        self.base_friction = 0.3 #minimum resistance drawer has (in kg)
        gpio.setmode(gpio.BCM) #sets pin mapping to GPIO pins
        gpio.setup(self.reset_pul, gpio.OUT)
        gpio.setup(self.reset_dir, gpio.OUT)
        gpio.setup(self.reset_en, gpio.OUT)
        gpio.setup(self.fric_pul, gpio.OUT)
        gpio.setup(self.fric_dir, gpio.OUT)
        gpio.setup(self.fric_en, gpio.OUT)
        self.reset_motor = StepperMotor(self.reset_pul, self.reset_dir, self.reset_en)
        self.fric_motor = StepperMotor(self.fric_pul, self.fric_dir, self.fric_en)
  
    def __set_friction(self):
        self.fric_motor.step(self.__resistance_steps, self.fric_motor.CW)
        gpio.output(self.fric_en, gpio.HIGH) #keep motor resistance on
      
    def __reset_friction(self):
        self.fric_motor.step_to(0)
  
    def reset_callback(self, goal):
        try:
	    self.reset_as.publish_feedback(StageFeedback(status="Resetting Friction"))
            self.__reset_friction()
            did_move = False
            end_pos = self.start_pos + self.dis_buffer
	    self.reset_as.publish_feedback(StageFeedback(status="Winding in Drawer"))
            while (True):
                if(self.tof.get_distance() <= end_pos):
                    break
                did_move = True
                self.reset_motor.move_for(0.1, self.reset_motor.CCW)
            if(did_move):
	        self.reset_as.publish_feedback(StageFeedback(status="Unwinding reset motor"))
                self.reset_motor.move_for(self.time_unwind, self.reset_motor.CW)
            self.tof.stop_ranging()
	    self.reset_as.set_succeeded(StageResult(result=0), text="SUCCESS")
        except:
	    self.reset_as.set_aborted(StageResult(result=100), text="FAILED")
    
    def parameters_callback(self, goal):
        resistance = goal.parameters[0]
        tof_mode = goal.parameters[1]
        try:
            self.tof_mode = tof_mode
            self.__resistance_steps = int(((resistance - self.base_friction) / self.fric_steps) + self.fric_min_steps)
            self.parameters_as.publish_feedback(TestParametersFeedback(status="getting starting position of drawer"))
            self.tof.start_ranging(self.tof_mode)
            self.start_pos = self.tof.get_distance()
            self.parameters_as.publish_feedback(TestParametersFeedback(status="setting friction to: {} N".format(resistance)))
            self.__set_friction()
            self.__trial_data = [] #not need?
	    self.parameters_as.set_succeeded(TestParametersResult(result=0), text="SUCCESS")
        except:
	    self.parameters_as.set_aborted(TestParametersResult(result=100), text="FAILED")

if __name__ == "__main__":
    rospy.init_node("reset", argv=sys.argv)
    initialize = HardwareReset()
    rospy.spin()
