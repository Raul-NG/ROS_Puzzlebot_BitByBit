#!/usr/bin/env python

import rospy
import numpy as np
from sensor_msgs.msg import Image
from geometry_msgs.msg import Pose2D
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
import numpy as np

class Line_Detector:
    def __init__(self):
        self.bridge = CvBridge()
        self.image_raw = None
        self.dt = 0.1

        rospy.init_node('Image_processor')
        rospy.Subscriber('/video_source/raw', Image, self.img_callback)
        self.edges_pub = rospy.Publisher('/img_properties/edges', Image, queue_size=10)
        self.lines_pub = rospy.Publisher('/img_properties/lines', Image, queue_size=10)
        self.rate = rospy.Rate(1/self.dt)
        self.timer = rospy.Timer(rospy.Duration(self.dt), self.timer_callback)
        # rospy.on_shutdown(self.stop)

    def timer_callback(self, time):
        self.detect_line()

    def img_callback(self,msg):
        self.image_raw = self.bridge.imgmsg_to_cv2(msg, "passthrough")

    def detect_line(self):
        # Grayscale and Canny Edges extracted
        gray_image = cv2.cvtColor(self.image_raw, cv2.COLOR_BGR2GRAY)

        _,msk = cv2.threshold(gray_image,120,255, cv2.THRESH_BINARY)
        erode = cv2.erode(msk, (5, 5), iterations = 6)

        Cutx = (int(3*gray_image.shape[0]/4),gray_image.shape[0])
        Cuty = (0,gray_image.shape[1])

        edges = cv2.Canny(erode, 0, 60)
        self.edges_pub.publish(self.bridge.cv2_to_imgmsg(edges))

        # Run HoughLines using a rho accuracy of 1 pixel
        # theta accuracy aof np.pi / 180 which is 1 degree
        # The line threshold is set to 240 (number of points on line)
        lines = cv2.HoughLines(edges[Cutx[0]:Cutx[1],Cuty[0]:Cuty[1]], 1, np.pi / 180, 50)

        # Iterate through each line and convert it to the format
        if lines is not None:
            for i in range(len(lines)): 
                rho, theta = lines[i,0,:]
                a = np.cos(theta)
                b = np.sin(theta)
                x0 = a * rho
                y0 = b * rho
                x1 = int(x0 + 1000 * (-b))
                y1 = int(y0 + 1000 * (a))
                x2 = int(x0 - 1000 * (-b))
                y2 = int(y0 - 1000 * (a))
                cv2.line(edges[Cutx[0]:Cutx[1],Cuty[0]:Cuty[1]], (x1, y1), (x2, y2), (127), 2)

        self.lines_pub.publish(self.bridge.cv2_to_imgmsg(edges[Cutx[0]:Cutx[1],Cuty[0]:Cuty[1]]))

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    line_detector = Line_Detector()
    try:
        line_detector.run()
    except:
        pass


