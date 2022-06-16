#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import numpy as np
import cv2

class Line_Detector:
    def __init__(self):
        self.bridge = CvBridge()
        self.image_raw = None
        self.dt = 0.1
        self.line = [0,0,0,0]
        self.lines = []
        self.error_horizontal = 0.0
        self.error_angle = 0.0
        self.error = 0.0
        # self.x_center = 640
        self.x_center = 440
        self.linear_speed = 0.12      #Linear velocity
        self.angular_speed = 0.0
        self.max_omega = np.pi/4    #Maximum angular velocity
        self.cut_y = (int(3*720.0/4.0),720)
        self.cut_x = (200,1080)
        # self.cut_x = (320,960)
        self.edges = None
        self.activate_flag = True
        self.intersection = 0

        rospy.init_node('Line_Detector')
        rospy.Subscriber('/video_source/raw', Image, self.img_callback)
        rospy.Subscriber('/activator', String, self.activator_callback)
        # rospy.Subscriber('/odom', Pose2D, self.odom_callback)
        self.edges_pub = rospy.Publisher('/img_properties/edges', Image, queue_size=10)
        self.lines_pub = rospy.Publisher('/img_properties/lines', Image, queue_size=10)
        # self.canny_1 = rospy.Publisher('/img_properties/canny_1', Image, queue_size=10)
        # self.canny_2 = rospy.Publisher('/img_properties/canny_2', Image, queue_size=10)
        self.move_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.talkback_pub = rospy.Publisher('/line_detector/talkback', String, queue_size=10)
        self.rate = rospy.Rate(1/self.dt)
        self.timer = rospy.Timer(rospy.Duration(self.dt), self.timer_callback)
        rospy.on_shutdown(self.stop)

    def timer_callback(self, time):
        self.lines = []
        if self.activate_flag and self.image_raw is not None:
            for _ in range(2):
                self.detect_lines()
            rospy.loginfo("Lineas: "+str(len(self.lines)))
            if len(self.lines) > (70 if self.intersection < 2 else 35):
                self.intersection += 1
                self.talkback_pub.publish(str(self.intersection))
            else:
                self.choose_line()
                self.navigate()
            self.show_lines()
    
    def activator_callback(self,msg):
        if msg.data == "LD_activate":
            self.activate_flag = True
        elif msg.data == "LD_deactivate":
            self.activate_flag = False
        elif msg.data == "no_speed_limit":
            self.linear_speed = 0.25
        elif msg.data == "speed_limit":
            self.linear_speed = 0.12
    def img_callback(self,msg):
        self.image_raw = self.bridge.imgmsg_to_cv2(msg, "passthrough")

    def detect_lines(self):
        # Grayscale and Canny Edges extracted
        gray_image = cv2.cvtColor(self.image_raw, cv2.COLOR_BGR2GRAY)
        # self.canny_1.publish(self.bridge.cv2_to_imgmsg(gray_image))
        _,msk = cv2.threshold(gray_image,90,255, cv2.THRESH_BINARY)
        # self.canny_2.publish(self.bridge.cv2_to_imgmsg(msk))
        msk = cv2.GaussianBlur(msk, (9,9), cv2.BORDER_DEFAULT)
        erode = cv2.erode(msk, np.ones((5, 5), np.uint8), iterations = 4)
        
        # self.edges = erode
        edges = cv2.Canny(erode, 150, 200)
        
        self.edges = cv2.dilate(edges, np.ones((5, 5), np.uint8), iterations = 1)
        self.edges_pub.publish(self.bridge.cv2_to_imgmsg(self.edges))
        linesP = cv2.HoughLinesP(self.edges[self.cut_y[0]:self.cut_y[1],self.cut_x[0]:self.cut_x[1]], 1, np.pi / 180, 50, None, 150, 80)
        if linesP is not None:
            self.lines.extend(linesP)
        # linesP = cv2.HoughLinesP(edges[Cutx[0]:Cutx[1],Cuty[0]:Cuty[1]], 1, np.pi / 180, 50, None, 100, 10)
        # linesP = cv2.HoughLinesP(edges[Cutx[0]:Cutx[1],Cuty[0]:Cuty[1]], 2, np.pi / 180, 70, None, 70, 10)
        
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

    def navigate(self):
        # kp = 0.05 * self.linear_speed
        # kd = 0.003 * self.linear_speed
        kp = 0.005 * self.linear_speed
        kd = 0.0003 * self.linear_speed
        x = self.line[0] if self.line[1] <= self.line[3] else self.line[2]
        error_ant = self.error
        self.error_horizontal = (self.x_center - x) #/ 640 * 3
        # if self.line[3] >= self.line[1]:
        #     angle = math.atan2(self.line[1]-self.line[3], self.line[0]-self.line[2])
        # else:
        #     angle = math.atan2(self.line[3]-self.line[1], self.line[2]-self.line[0])
        # self.error_angle = angle - np.pi/2 if abs(angle - np.pi/2) > 0.4 else 0.0
        # self.error = self.error_angle + self.error_horizontal
        self.error = self.error_horizontal
        self.angular_speed = kp*self.error + kd*(self.error - error_ant)/self.dt

        msg = Twist()
        msg.linear.x = self.linear_speed
        msg.angular.z = self.angular_speed
        self.move_pub.publish(msg)
    
    def check_trff_lgt(self):
        t = Twist()
        t.linear.x = 0.0
        t.angular.z = 0.0
        self.move_pub.publish(t)


    def run(self):
        rospy.spin()
    
    def stop(self):
        rospy.loginfo("Stopping line detection.")
        self.timer.shutdown()
        t = Twist()
        t.linear.x = 0.0
        t.angular.z = 0.0
        self.move_pub.publish(t)

if __name__ == '__main__':
    line_detector = Line_Detector()
    try:
        line_detector.run()
    except:
        pass


