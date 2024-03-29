#!/usr/bin/env python

import rospy
import sys
import actionlib
import time
import roslaunch
import numpy as np
import cv2 as cv
import os, subprocess, shlex, signal
from infrastructure_msgs.msg import StageAction, StageGoal, StageFeedback, StageResult, DataTimestamps




class DataCollection():
	
	def __init__(self):   
		#initializing actionservers
		self.start_collection = actionlib.SimpleActionServer("start_data_collection", StageAction, self.start_collection_callback, False) 
		self.stop_collection = actionlib.SimpleActionServer("stop_data_collection", StageAction, self.stop_collection_callback, False) 
		self.start_collection.start()
		self.stop_collection.start()

                self.time_stamp = DataTimestamps()
                self.time_stamp_pub = rospy.Publisher("hardware_timestamps", DataTimestamps, queue_size=10)
		self.collection_flag = False
		self.trial_count = 0
		self.video_path = os.path.dirname(os.path.realpath(__file__))
		self.video_path = os.path.split(self.video_path)[0] + "/stored_data/recorded_video/"
		self.name_parameter = rospy.get_param("test_name", "infrastructure_trial_")
		record_parameter = rospy.get_param("record_video", False)
		
		self.record_rosbags = rospy.get_param("record_trial_rosbags", False)
		self.robot_jointState_topic =  rospy.get_param("robot_joints", "/")

		# create directory for rosbag(s) to be stored in. stores data in parent dir to catkin ws
		if(self.record_rosbags):
			parent_dir = os.path.abspath('..')
			self.rosbags_dir = parent_dir + "/stored_data/rosbags/"
			if(not os.path.exists(self.rosbags_dir)):
				os.makedirs(self.rosbags_dir)

		if(record_parameter == True):
			while(not rospy.is_shutdown()):
				if(self.collection_flag == True):
					cap = cv.VideoCapture(0)
					fourcc = cv.VideoWriter_fourcc(*'XVID')
					name = str(self.video_path) + self.name_parameter + str(self.trial_count) + ".avi"
					out = cv.VideoWriter(name, fourcc, 30, (640, 480))

					self.start_collection.publish_feedback(StageFeedback(status="STARTING RECORDING"))
					while(self.collection_flag == True and cap.isOpened() and not rospy.is_shutdown()):
                                                ret, frame = cap.read()
						if(not ret):
							print("ERROR, CANT GET FRAME")
							break
						out.write(frame)
					self.stop_collection.publish_feedback(StageFeedback(status="STOPPING RECORDING"))
					cap.release()
					out.release()

				else:
					time.sleep(1)


		 
	def start_collection_callback(self, goal):

		if(self.collection_flag == True):
			self.start_collection.set_aborted(StageResult(result = 100), text="ABORT, PREVIOUS DATA COLLECTION WAS NOT STOPPED")
			return

		self.collection_flag = True
		self.trial_count += 1

		#used for hardware controller
                time.sleep(3)
                self.time_stamp.trial_number = self.trial_count
                self.time_stamp.collection_start_time = rospy.Time.now()
		
		# start rosbag recording
		if(self.record_rosbags):
			self.rosbag_name = self.rosbags_dir + self.name_parameter + "_trial_" + str(self.trial_count)
			tokenized_args = shlex.split("rosbag record -O " + self.rosbag_name + " -e '(.*)_infsensor' " + self.robot_jointState_topic)
			self.rosbag = subprocess.Popen(tokenized_args)

		self.start_collection.set_succeeded(StageResult(result = 0), text="SUCCESS")
			


	def stop_collection_callback(self, goal):

		if(self.collection_flag == False):
			self.stop_collection.set_aborted(StageResult(result = 100), text="ABORT, DATA COLLECTION WAS NOT STARTED")
			return
		
		# stop rosbag recording
		if(self.record_rosbags):
			self.rosbag.send_signal(signal.SIGINT)

		self.collection_flag = False
                self.time_stamp.collection_end_time = rospy.Time.now()
                self.time_stamp.total_time = self.time_stamp.collection_end_time.secs - self.time_stamp.collection_start_time.secs
                self.time_stamp_pub.publish(self.time_stamp)
		#time.sleep(5)
		self.stop_collection.set_succeeded(StageResult(result = 0), text="SUCCESS")
  



if __name__ == '__main__':
	rospy.init_node("data_collection", argv=sys.argv)
	begin = DataCollection()
	rospy.spin()
