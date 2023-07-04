import subprocess
import yaml
import rosbag
import cv2
from cv_bridge import CvBridge
import numpy as np
from copy import deepcopy
import skimage.exposure


FILENAME = 'grasp_dataset_x_trans_top_trial_1'
ROOT_DIR = '/root/stored_data/rosbags'
BAGFILE = ROOT_DIR + '/' + FILENAME + '.bag'

if __name__ == '__main__':
    bag = rosbag.Bag(BAGFILE)

   
    #if (i == 0):
    #    TOPIC = '/camera/depth/image_rect_raw'
    #    DESCRIPTION = 'depth_'
    img_array = []
    
    TOPIC = '/mounted_camera_4/compressed'#/camera/color/image_raw/compressed'
    DESCRIPTION = 'color_'
    image_topic = bag.read_messages(TOPIC)
    #print(image_topic)
    size = (640,480)
    fourcc = cv2.VideoWriter_fourcc(*'MJPG')
    cv2.namedWindow('image', flags=cv2.WINDOW_AUTOSIZE)
    #out = cv2.VideoWriter('/root/infrastructure_ws/test_final.avi', fourcc, 30, size)
    for k, b in enumerate(image_topic):
        bridge = CvBridge()
        cv_image = bridge.compressed_imgmsg_to_cv2(b.message, desired_encoding='passthrough') #compressed_imgmsg_to_cv2
        #cv_image.astype(np.uint8)
        #if (DESCRIPTION == 'depth_'):
        #    depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(cv_image, alpha=0.03), cv2.COLORMAP_JET)
        #    cv2.imwrite(ROOT_DIR + '/depth/' + DESCRIPTION + str(b.timestamp) + '.png', cv_image)
                    #cv2.imwrite(ROOT_DIR + '/color/' + DESCRIPTION + str(b.timestamp) + '.png', cv_image)
        #imC = skimage.exposure.rescale_intensity(cv_image, in_range='image', out_range=(0,255)).astype(np.uint8)

        
        cv2.imshow('image',cv_image)
        cv2.waitKey(1)
        #print('saved: ' + DESCRIPTION + str(b.timestamp) + '.png')   
    

        #out.write(cv_image)
    #out.release()


    bag.close()

    print('PROCESS COMPLETE')
