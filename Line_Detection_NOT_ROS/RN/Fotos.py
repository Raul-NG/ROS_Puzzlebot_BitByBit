#!/usr/bin/env python

import cv2
import rospy
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Image 
from std_msgs.msg import Float32
from std_msgs.msg import Float32MultiArray
from cv_bridge import CvBridge

class Fotos:

    def __init__(self):
        self.bridge = CvBridge()

        self.image_raw = None
        self.count = 0
        
        self.dt = 0.2
        rospy.init_node('traffic_light_detector')

        self.signals = ['stop', 'continue', 'round','turn', 'no speed limit']
        self.rate = rospy.Rate(1/self.dt)
        rospy.Subscriber('/video_source/raw', Image, self.img_callback)
        rospy.on_shutdown(self.stop)        

    def img_callback(self,msg):
        if self.count < 200:
            self.image_raw = self.bridge.imgmsg_to_cv2(msg, "passthrough")
            cv2.imwrite("stop/stop_%d.jpg" % self.count, self.image_raw)
            self.count += 1
        # cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='CV_8UC3')
            
    def run(self):
        while True:
            pass
            

    def stop(self):
        rospy.loginfo("Stopping traffic light detector.")

if __name__ == '__main__':
    fotos = Fotos()
    try:
        fotos.run()
    except:
        pass
