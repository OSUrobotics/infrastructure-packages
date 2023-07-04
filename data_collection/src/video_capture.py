#!/usr/bin/env python

import rospy
import sys
import actionlib
import time
import roslaunch
import numpy as np
import cv2
from cv_bridge import CvBridge
import os, subprocess, shlex, signal

from sensor_msgs.msg import CompressedImage




class Camera():
    
    def __init__(self):   
        #initializing actionservers
        id = rospy.get_param("camera_id", 0)
        print("ID: ", id)

        rospy.init_node("cam_"+str(id), argv=sys.argv)
        print("started video node")


        self.br = CvBridge()
        self.loop_rate = rospy.Rate(30)

        # Publish the images
        self.pub = rospy.Publisher('mounted_camera_'+str(id)+'/compressed', CompressedImage,queue_size=10)
        self.publish_video(id)
        rospy.sleep(4)

    def publish_video(self, id):
        cam = cv2.VideoCapture(id)
        counter = 0
        # Publishes video
        while not rospy.is_shutdown():
            ret, frame = cam.read()
            if not ret: 
                counter += 1
                if counter > 10:
                    err_str = 'Failed to get video for camera ' + str(id)
                    rospy.logerr(err_str)
                continue

            msg = CompressedImage()
            msg.header.stamp = rospy.Time.now()
            msg.format = "jpeg"
            msg.data = np.array(cv2.imencode('.png', frame)[1]).tostring()
            counter = 0
            # Get the image
            self.pub.publish(msg)
            self.loop_rate.sleep()
        
  



if __name__ == '__main__':
    begin = Camera()
    
