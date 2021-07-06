#!/usr/bin/env python

import rospy
import sys
import actionlib
from infrastructure_msgs.msg import TestParametersAction, TestParametersGoal, TestParametersFeedback, TestParametersResult
import hardware_setup

class HardwareParameters():
    
    def __init__(self):
        hardware_setup.init()
	self.parameters_as = actionlib.SimpleActionServer("set_test_parameters", TestParametersAction, self.parameters_callback, False) 
        self.parameters_as.start()
 
    def parameters_callback(self, goal):
        friction_setting = goal.parameters[0]
        try:
            self.parameters_as.publish_feedback(TestParametersFeedback(status="setting friction to: {} N".format(friction_setting)))
            hardware_setup.drawer.start_new_trial(friction_setting)
	    self.parameters_as.set_succeeded(TestParametersResult(result=0), text="SUCCESS")
        except:
	    self.parameters_as.set_aborted(TestParametersResult(result=100), text="FAILED")


if __name__ == '__main__':
    rospy.init_node("test_parameters", argv=sys.argv)
    begin = HardwareParameters()
    rospy.spin()
