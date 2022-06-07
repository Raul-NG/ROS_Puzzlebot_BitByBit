#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import UInt8
from std_msgs.msg import String
from cv_bridge import CvBridge
import numpy as np
import cv2

class Intersection_Detector:
    def __init__(self):
        self.bridge = CvBridge()
        self.image_raw = None
        self.dt = 0.1
        self.cut_y = (int(3*720.0/4.0),720)
        self.cut_x = (0,1280)
        self.activate = False
        # self.cut_x = (320,960)
        


        rospy.init_node('intersection_detector')
        rospy.Subscriber('/video_source/raw', Image, self.img_callback)
        # rospy.Subscriber('/activator', String, self.act_callback)
        # rospy.Subscriber('/odom', Pose2D, self.odom_callback)
        self.image_blob = rospy.Publisher('/img_properties/image_blob', Image, queue_size=10)
        self.blobs_pub = rospy.Publisher('/img_properties/blobs', UInt8, queue_size=10)
        self.timer = rospy.Timer(rospy.Duration(self.dt), self.timer_callback)
        rospy.on_shutdown(self.stop)

    def timer_callback(self, time):
        # if not self.activate:
        #     return

        for _ in range(2):
            self.detect_blobs()

    def img_callback(self,msg):
        self.image_raw = self.bridge.imgmsg_to_cv2(msg, "passthrough")

    def act_callback(self,msg):
        name = msg.data
        if name == 'interD_activate':
            self.activate = True
        elif name =='interD_deactivate':
            self.activate = False

    
    def detect_blobs(self):

        gray_image = cv2.cvtColor(self.image_raw[self.cut_y[0]:self.cut_y[1],self.cut_x[0]:self.cut_x[1]], cv2.COLOR_BGR2GRAY)

        params = cv2.SimpleBlobDetector_Params()
        # Set Area filtering parameters
        params.filterByArea = True
        params.minArea = 100
        # Set Circularity filtering parameters
        params.filterByCircularity = True 
        params.minCircularity = 0.4
        params.maxCircularity = 0.7
        # Set Convexity filtering parameters 
        params.filterByConvexity = False
        # Set inertia filtering parameters
        params.filterByInertia = True
        params.minInertiaRatio = 0.01
        # Create a detector with the parameters
        detector = cv2.SimpleBlobDetector_create(params)
        keypoints = detector.detect(gray_image)

        number_of_blobs = len(keypoints)
        self.blobs_pub.publish(number_of_blobs)
        blank = np.zeros((1,1)) 
        blobs = cv2.drawKeypoints(gray_image, keypoints, blank, (0,0,255),cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
        # blobs = cv2.cvtColor(blobs, cv2.COLOR_GRAY2BGR)
        self.image_blob.publish(self.bridge.cv2_to_imgmsg(blobs))
    
    def run(self):
        rospy.spin()
    
    def stop(self):
        rospy.loginfo("Stopping intersection detection.")
        self.timer.shutdown()

if __name__ == '__main__':
    intersection_detector = Intersection_Detector()
    try:
        intersection_detector.run()
    except:
        pass
