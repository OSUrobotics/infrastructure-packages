import subprocess
import yaml
import rosbag
import cv2
from cv_bridge import CvBridge
import numpy as np
from copy import deepcopy


FILENAME = 'ahhh3_trial_1'
ROOT_DIR = '/root/stored_data/rosbags'
BAGFILE = ROOT_DIR + '/' + FILENAME + '.bag'

if __name__ == '__main__':
    bag = rosbag.Bag(BAGFILE)

   
    #if (i == 0):
    #    TOPIC = '/camera/depth/image_rect_raw'
    #    DESCRIPTION = 'depth_'
    img_array = []
    
    TOPIC = '/camera_2'
    DESCRIPTION = 'color_'
    image_topic = bag.read_messages(TOPIC)
    size = (640,480)
    fourcc = cv2.VideoWriter_fourcc(*'MJPG')
    out = cv2.VideoWriter('/root/infrastructure_ws/test_final.avi', fourcc, 30, size)
    for k, b in enumerate(image_topic):
        bridge = CvBridge()
        cv_image = bridge.compressed_imgmsg_to_cv2(b.message, desired_encoding='passthrough')
        #cv_image.astype(np.uint8)
        #if (DESCRIPTION == 'depth_'):
        #    depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(cv_image, alpha=0.03), cv2.COLORMAP_JET)
        #    cv2.imwrite(ROOT_DIR + '/depth/' + DESCRIPTION + str(b.timestamp) + '.png', cv_image)
                    #cv2.imwrite(ROOT_DIR + '/color/' + DESCRIPTION + str(b.timestamp) + '.png', cv_image)
        cv2.imshow('image',cv_image)
        cv2.waitKey(1)
        #print('saved: ' + DESCRIPTION + str(b.timestamp) + '.png')   
    

        out.write(cv_image)
    out.release()


    bag.close()

    print('PROCESS COMPLETE')
