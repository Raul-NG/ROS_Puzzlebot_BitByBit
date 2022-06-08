#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import numpy as np
import cv2
import math

class Traffic_Light_Detector:
    def __init__(self):
        self.activate = False
        self.bridge = CvBridge()
        self.image_raw = None
        self.dt = 0.1
        self.cut_y = (int(3*720.0/4.0),720)
        self.cut_x = (0,1280)
        self.tem = []

        self.tml = 7
        self.orb = cv2.ORB_create(1000)
        self.orb.setScaleFactor(1.2)
        self.flann = cv2.FlannBasedMatcher(dict(algorithm = 0, trees = 3), dict(checks = 100))
        self.dest = []
        self.kps = []
        for i in range(1,self.tml + 1):
            kptemp, destemp = self.orb.detectAndCompute(cv2.cvtColor(cv2.imread('/home/puzzlebot/catkin_ws/src/traffic_light_detection/src/semaforo_t_%d.png' % i), cv2.COLOR_BGR2GRAY), None)
            self.kps.append(kptemp)
            self.dest.append(np.float32(destemp))
        self.colores = ['verde', 'amarillo', 'rojo','apagado 1', 'apagado 2','verde','rojo']
        self.slml = []
        self.il = []
        self.slma = 0
        self.ila = 0

        rospy.init_node('Traffic_Light_Detector')
        rospy.Subscriber('/video_source/raw', Image, self.img_callback)
        rospy.Subscriber('/activator', String, self.activator_callback)
        self.traffic_light_pub = rospy.Publisher('/traffic_light', String, queue_size=10)
        self.kp_pub = rospy.Publisher('/keypoints1', Image, queue_size=10)
        self.rate = rospy.Rate(1/self.dt)
        self.timer = rospy.Timer(rospy.Duration(self.dt), self.timer_callback)
        rospy.on_shutdown(self.stop)

    def timer_callback(self, time):
        self.kp_pub.publish(self.bridge.cv2_to_imgmsg(cv2.drawKeypoints(cv2.imread('/home/puzzlebot/catkin_ws/src/traffic_light_detection/src/semaforo_t_1.png'),self.kps[0],cv2.imread('/home/puzzlebot/catkin_ws/src/traffic_light_detection/src/semaforo_t_1.png'), flags=cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS), "bgr8"))
        
        if not self.activate: 
            return
        self.check_trff_lgt()
        
    def check_trff_lgt(self):
        _, desf = self.orb.detectAndCompute(cv2.cvtColor(self.image_raw, cv2.COLOR_BGR2GRAY), None)
        desf = np.float32(desf)
        slm = 0
        index = 0
        matches = []
        matchesMask = []
        for j in range(self.tml):
            matches.append(self.flann.knnMatch(self.dest[j], desf, k=2))
            matchesMask.append([[0,0] for k in range(len(matches[j]))])
            for k,(m,n) in enumerate(matches[j]):
                if m.distance < 0.7*n.distance:
                    matchesMask[j][k]=[1,0]
            matchesMask[j] = np.array(matchesMask[j])
            if slm < np.sum(matchesMask[j][:,0]):
                slm = np.sum(matchesMask[j][:,0])
                index = j
        self.slml.append(slm)
        self.il.append(index)
        if len(self.slml) > 9:
            self.slml.pop(0)
            self.il.pop(0)
            self.slma = round(np.median(np.array(self.slml)))
            self.slmm = np.max(np.array(self.slml))
            self.ila = self.il[self.slml.index(self.slmm)]
        if self.slma > 3:
            # Mandar colores[ila]
            if self.colores[self.ila] == "rojo":
                self.traffic_light_pub.publish("Rojo")
            if self.colores[self.ila] == "verde":
                self.traffic_light_pub.publish("Verde")
            else:
                self.traffic_light_pub.publish("Detecta algo")
        else:        
            self.traffic_light_pub.publish("No detecta nada")
    
    def img_callback(self,msg):
        self.image_raw = self.bridge.imgmsg_to_cv2(msg, "passthrough")

    def activator_callback(self,msg):
        if msg.data == "TL_activate":
            self.activate = True
        elif msg.data == "TL_deactivate":
            self.activate = False

    def run(self):
        rospy.spin()
    
    def stop(self):
        rospy.loginfo("Stopping traffic light detector.")

if __name__ == '__main__':
    traffic_light_detector = Traffic_Light_Detector()
    try:
        traffic_light_detector.run()
    except:
        pass


