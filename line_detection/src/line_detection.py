#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from std_msgs.msg import Float32MultiArray
from cv_bridge import CvBridge
import numpy as np
import cv2

class Line_Detector:
    def __init__(self):
        self.bridge = CvBridge()
        self.x_center = 640
        self.image_raw = None
        self.dt = 0.3
        self.line = [0,0,0,0]
        self.lines = []
        self.cut_y = (int(3*720.0/4.0),720)
        self.cut_x = (0,1280)
        # self.cut_x = (320,960)
        self.edges = None
        self.activate = False

        rospy.init_node('Line_Detector')
        rospy.Subscriber('/video_source/raw', Image, self.img_callback)
        rospy.Subscriber('/activator', String, self.activator_callback)
        # rospy.Subscriber('/odom', Pose2D, self.odom_callback)
        self.edges_pub = rospy.Publisher('/img_properties/edges', Image, queue_size=10)
        self.lines_pub = rospy.Publisher('/img_properties/lines', Image, queue_size=10)
        self.line_detector_pub = rospy.Publisher('/line_detector', Float32MultiArray, queue_size=10)
        self.rate = rospy.Rate(1/self.dt)
        self.timer = rospy.Timer(rospy.Duration(self.dt), self.timer_callback)
        rospy.on_shutdown(self.stop)

    def img_callback(self,msg):
        self.image_raw = self.bridge.imgmsg_to_cv2(msg, "passthrough")

    def activator_callback(self,msg):
        if msg.data == "LD_activate":
            self.activate = True
        elif msg.data == "LD_deactivate":
            self.activate = False

    def timer_callback(self, time):
        if not self.activate:
            return
        
        self.lines = []
        msg = Float32MultiArray()
        for _ in range(2):
            self.detect_lines()

        if len(self.lines) > 70:
            msg.data = [-1, -1, -1, -1]
            self.line_detector_pub.publish(msg)
        else:
            self.choose_line()
            msg.data = [self.line[i] for i in range(4)]
            self.line_detector_pub.publish(msg)

        self.show_lines()

    def detect_lines(self):
        # Grayscale and Canny Edges extracted
        gray_image = cv2.cvtColor(self.image_raw, cv2.COLOR_BGR2GRAY)
        _,msk = cv2.threshold(gray_image,90,255, cv2.THRESH_BINARY)
        msk = cv2.GaussianBlur(msk, (9,9), cv2.BORDER_DEFAULT)
        erode = cv2.erode(msk, np.ones((5, 5), np.uint8), iterations = 4)
        
        # self.edges = erode
        edges = cv2.Canny(erode, 150, 200)
        
        self.edges = cv2.dilate(edges, np.ones((5, 5), np.uint8), iterations = 1)
        self.edges_pub.publish(self.bridge.cv2_to_imgmsg(self.edges))
        linesP = cv2.HoughLinesP(self.edges[self.cut_y[0]:self.cut_y[1],self.cut_x[0]:self.cut_x[1]], 1, np.pi / 180, 50, None, 100, 80)
        if linesP is not None:
            self.lines.extend(linesP)
        
    def choose_line(self):
        # for i in range(0, len(linesP)):
        #     max_len = 0
        #     l = linesP[i][0]
        #     line_len = np.sqrt((l[0]-l[2])**2 + (l[1]-l[3])**2)
        #     if line_len > max_len:
        #         max_len = line_len
        #         self.line = l
        #     cv2.line(edges[Cutx[0]:Cutx[1],Cuty[0]:Cuty[1]], (l[0], l[1]), (l[2], l[3]), 127, 3, cv2.LINE_AA)
        error = 1280
        for line in self.lines:
            l = line[0]
            x_top = l[0] if l[1] <= l[3] else l[2]
            if abs(x_top - self.x_center) < error:
                error = abs(x_top - self.x_center)
                self.line = l
    
    def show_lines(self):
        edges = self.edges[self.cut_y[0]:self.cut_y[1],self.cut_x[0]:self.cut_x[1]]
        for line in self.lines:
            l = line[0]
            cv2.line(edges, (l[0], l[1]), (l[2], l[3]), 100, 3, cv2.LINE_AA)
        cv2.line(edges, (self.line[0], self.line[1]), (self.line[2], self.line[3]), 200, 3, cv2.LINE_AA)
        self.lines_pub.publish(self.bridge.cv2_to_imgmsg(edges))

    def run(self):
        rospy.spin()
    
    def stop(self):
        rospy.loginfo("Stopping line detection.")
        self.timer.shutdown()

if __name__ == '__main__':
    line_detector = Line_Detector()
    try:
        line_detector.run()
    except:
        pass


