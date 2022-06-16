#!/usr/bin/env python

import cv2
import rospy
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class Traffic_Light_Detector:

    def __init__(self):
        self.activate = True #False
        self.time_sleep = True
        self.bridge = CvBridge()

        self.image_raw = None

        # self.cut_y = (0,int(720.0/2.0))
        # self.cut_x = [(0,int(1280.0/3.0)), (int(1280.0/3.0),int(1280.0/3.0*2.0))]
        self.cut_y = (0,int(2*720.0/3.0))
        self.cut_x = [(0,int(1280.0/3.0))]#, (int(1280.0/2.0),int(1280.0/3.0*2.0))]
        
        self.erode = None
        self.dt = 0.2
        self.index = -1

        rospy.init_node('traffic_light_detector')
        rospy.Subscriber('/activator', String, self.activator_callback)
        rospy.Subscriber('/video_source/raw', Image, self.img_callback)

        self.colors = ['red','green','yellow','color not detected']
        # self.mask_publishers = [[rospy.Publisher('/traffic_light/first/'+color, Image, queue_size=10) for color in self.colors[:3]], [rospy.Publisher('/traffic_light/second/'+color, Image, queue_size=10) for color in self.colors[:3]]]
        # self.traffic_light_pub = [rospy.Publisher('/traffic_light/first/detection', String, queue_size=10),
        #                         rospy.Publisher('/traffic_light/second/detection', String, queue_size=10)]
        self.mask_publishers = [[rospy.Publisher('/traffic_light/'+color, Image, queue_size=10) for color in self.colors[:3]]]
        self.traffic_light_pub = [rospy.Publisher('/traffic_light/detection', String, queue_size=10)]

        self.rate = rospy.Rate(1/self.dt)
        rospy.on_shutdown(self.stop)
        self.timer = rospy.Timer(rospy.Duration(self.dt), self.timer_callback)

    def timer_callback(self, time):
        # self.time_sleep = True
        if self.activate and self.image_raw is not None:
            self.time_sleep = False
            self.color_check(0)
            # self.color_check(1)

    def img_callback(self,msg):
        self.image_raw = self.bridge.imgmsg_to_cv2(msg, "passthrough")
        # cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='CV_8UC3')

    def activator_callback(self,msg):
        if msg.data == "TL_activate":
            self.activate = True
        elif msg.data == "TL_deactivate":
            self.activate = False

    def color_check(self,semaforo_num):
        hsv = cv2.cvtColor(self.image_raw, cv2.COLOR_BGR2HSV)#[self.cut_y[0]:self.cut_y[1],self.cut_x[semaforo_num][0]:self.cut_x[semaforo_num][1]]
        masks = [cv2.add(cv2.inRange(hsv, (0, 100, 20), (8, 255,255)), cv2.inRange(hsv, (175, 100, 20), (180, 255,255))), # red
                cv2.inRange(hsv, (40, 40, 40), (115, 255,255)), # green
                cv2.inRange(hsv, (15, 100, 100), (30, 255,255))] # yellow
        self.index = -1;
        den_ant = 0
        # msk = cv2.erode(masks[1], np.array([[0,1,0],[0,1,1],[0,1,1]], np.uint8), iterations = 4)
        msk = cv2.erode(masks[1], np.ones((3, 3)), iterations = 4)
        msk = cv2.dilate(msk, np.ones((3, 3)), iterations = 4)
        self.mask_publishers[semaforo_num][1].publish(self.bridge.cv2_to_imgmsg(msk))
        den = np.sum(msk)/((self.cut_x[semaforo_num][1]-self.cut_x[semaforo_num][0])*(self.cut_y[1] - self.cut_y[0])*255)
        # rospy.loginfo("Den "+self.colors[color]+": "+str(den))
        if den > 0.002:
            self.traffic_light_pub[semaforo_num].publish(self.colors[1])
        else:
            self.traffic_light_pub[semaforo_num].publish(self.colors[0])
        # for color, mask in enumerate(masks):
        #     erode = cv2.erode(mask, np.array([[0,1,0],[0,1,1],[0,1,1]], np.uint8), iterations = 2)
        #     # self.dilate = cv2.dilate(erode, np.ones((3, 3)), iterations = 4)
        #     self.mask_publishers[semaforo_num][color].publish(self.bridge.cv2_to_imgmsg(erode))
        #     den = np.sum(erode)/((self.cut_x[semaforo_num][1]-self.cut_x[semaforo_num][0])*(self.cut_y[1] - self.cut_y[0])*255)
        #     # rospy.loginfo("Den "+self.colors[color]+": "+str(den))
        #     if den > den_ant and den > 0.002:
        #         den_ant = den
        #         self.index = color
        # self.traffic_light_pub[semaforo_num].publish(self.colors[self.index])
            
    def run(self):
        rospy.spin()

    def stop(self):
        rospy.loginfo("Stopping traffic light detector.")

if __name__ == '__main__':
    traffic_light_detector = Traffic_Light_Detector()
    # try:
    traffic_light_detector.run()
    # except:
    #     pass