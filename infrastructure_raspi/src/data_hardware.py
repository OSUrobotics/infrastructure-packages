#!/usr/bin/env python


# Main controller node
#
# drawer_controller.py
#
# Ryan Roberts
#
# A node that handles all of the Action servers and topics for the Drawer

import rospy
import sys
import actionlib
import RPi.GPIO as gpio
#import hardware_setup

#messages
from infrastructure_msgs.msg import StageActionResult
from infrastructure_msgs.msg import DoorSensors

class HardwareDataCollection():

    def __init__(self):
        # TEMPORARY SOLUTION (should be replaced with data_collection action server)
        # publisher that eavesdrops on data collection action server result topics and publishes to a topic
        # that gets recorded in a rosbag.
        start_sub = rospy.Subscriber("start_data_collection/result", StageActionResult, self.start_data_callback)
        stop_sub = rospy.Subscriber("stop_data_collection/result", StageActionResult, self.stop_data_callback)
        data_pub = rospy.Publisher("hardware_infsensor", DoorSensors, queue_size=10)
        rate = rospy.Rate(32)
        self.collect_data = False
        
        while not rospy.is_shutdown():
            if(self.collect_data):
                #collect data
                data_message = DoorSensors()
                #format message. can be more optimized (and more feedback messages) if we integrate hardware class with #controller
                data_point = hardware_setup.drawer.collect_data()
                data_message.current_time = rospy.Time.now()
                data_message.tof = data_point.tof
                #computationally faster than using for loop with string concatenation
                data_message.fsr1 = data_point.handle_data[0]
                data_message.fsr2 = data_point.handle_data[1]
                data_message.fsr3 = data_point.handle_data[2]
                data_message.fsr4 = data_point.handle_data[3]
                data_message.fsr5 = data_point.handle_data[4]
                data_message.fsr6 = data_point.handle_data[5]
                data_message.fsr7 = data_point.handle_data[6]
                data_message.fsr8 = data_point.handle_data[7]
                data_message.fsr9 = data_point.handle_data[8]
                data_message.fsr10 = data_point.handle_data[9]
                data_message.fsr11 = data_point.handle_data[10]
                data_message.fsr12 = data_point.handle_data[11]
                data_message.fsr_contact_1 = -1
                data_message.fsr_contact_2 = -1
                data_pub.publish(data_message)
                rate.sleep()

        #runs after node is closed. can be placed somewhere else
        gpio.cleanup() 

    def start_data_callback(self, msg):
        self.collect_data = True

    def stop_data_callback(self, msg):
        self.collect_data = False

if __name__ == "__main__":
    rospy.init_node("data_hardware", argv=sys.argv)
    initialize = HardwareDataCollection()
