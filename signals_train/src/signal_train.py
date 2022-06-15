#!/usr/bin/env python

import cv2
import rospy
import time
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class Fotos:

    def __init__(self):
        self.bridge = CvBridge()

        self.image_raw = None
        self.count = 0
        self.edges = None
        self.dt = 0.2
        rospy.init_node('signal_train')

        self.signals = ['stop', 'continue', 'round','turn', 'no speed limit']
        self.rate = rospy.Rate(1/self.dt)
        rospy.Subscriber('/video_source/raw', Image, self.img_callback)
        rospy.on_shutdown(self.stop)        

    def img_callback(self,msg):
        self.image_raw = self.bridge.imgmsg_to_cv2(msg, "passthrough")
        # gray1 = cv2.cvtColor(self.image_raw, cv2.COLOR_BGR2GRAY)
        # self.edges = cv2.Canny(np.uint8(gray1), 85, 70)    #proprocessing 

        # cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='CV_8UC3')
            
    def run(self):
        for count in range(200):
            time.sleep(0.1)
            cv2.imwrite("/home/puzzlebot/catkin_ws/Signals/stop_%d.jpg" % count, self.image_raw)
            

    def stop(self):
        rospy.loginfo("Stopping traffic light detector.")

if __name__ == '__main__':
    fotos = Fotos()
    try:
        fotos.run()
    except:
        pass
