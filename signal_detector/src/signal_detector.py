#!/usr/bin/env python

from cv2 import KalmanFilter
import rospy
from std_msgs.msg import String
from std_msgs.msg import Int32
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import numpy as np
import cv2
import math

class Signal_Detector:
    def __init__(self):
        self.activate = False
        self.bridge = CvBridge()
        self.image_raw = None
        self.dt = 0.07
        self.cut_y = (int(3*720.0/4.0),720)
        self.cut_x = (0,1280)
        self.tem = []

        #Kalman Filter Variables
        self.signals = ['no matches','continue','turn','round','no speed limit','stop']
        self.a = 0.9
        self.s_pred = 0
        self.sigma_u = 1
        self.M = 0
        self.s_hat = []

        self.tml = 5
        self.orb = cv2.ORB_create(500)
        self.orb2 = cv2.ORB_create(1000)
        self.flann = cv2.FlannBasedMatcher(dict(algorithm = 0, trees = 3), dict(checks = 1000))
        self.dest = []
        for i in range(self.tml):
            self.tem.append(cv2.imread('/home/puzzlebot/catkin_ws/src/signal_detector/src/Signal_%d.jpeg' % i))
            self.tem[i] = np.pad(cv2.pyrDown(self.tem[i]), pad_width=[(50, 50),(50, 50),(0, 0)], mode='constant',constant_values=(255))
            _, destemp = self.orb.detectAndCompute(self.tem[i], None)
            self.dest.append(np.float32(destemp))
        # self.signals = ['stop', 'continue', 'round','turn', 'no speed limit','no']
        self.slml = []
        self.il = []
        self.ila = ""
        self.slmp = 0
        self.sizel = 15

        rospy.init_node('Signal_Detector')
        rospy.Subscriber('/video_source/raw', Image, self.img_callback)
        rospy.Subscriber('/activator', String, self.activator_callback)
        self.signal_pub = rospy.Publisher('/signal', String, queue_size=10)
        self.index_pub = rospy.Publisher('/index/signal', Int32, queue_size=10)
        self.kf_pub = rospy.Publisher('/index/kf', Int32, queue_size=10)
        self.rate = rospy.Rate(1/self.dt)
        self.timer = rospy.Timer(rospy.Duration(self.dt), self.timer_callback)
        rospy.on_shutdown(self.stop)

    def timer_callback(self, time):
        # if not self.activate: 
        #     return
        self.check_trff_lgt()
        
    def check_trff_lgt(self):
        _, desf = self.orb.detectAndCompute(self.image_raw, None)
        desf = np.array(np.float32(desf))
        slm = 0
        signal_index = 0
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
                signal_index = j+1
        self.index_pub.publish(signal_index)
        self.kalman_filter(signal_index)
        self.kf_pub.publish(round(self.s_pred))
        self.signal_pub.publish(self.signals[int(round(self.s_pred))])
        # self.slml.append(slm)
        # self.il.append(index)
        # if len(self.slml) > self.sizel:
        #     self.slml.pop(0)
        #     self.il.pop(0)
        # self.slmp = np.mean(np.array(self.slml))
        # vals, counts = np.unique(self.il, return_counts=True)
        # self.ila = self.signals[int(vals[np.argwhere(counts == np.max(counts))][0])]
        # self.slma = round(np.median(np.array(self.slml)))
        # if self.slmp >= 2.0:
            # Mandar signals[self.ila]
        



        # if self.ila == "stop" and self.slmp >= 3.0:
        #     self.signal_pub.publish("stop, "+"slmp: "+str(self.slmp))
        # elif self.ila == "continue":
        #     self.signal_pub.publish("continue, "+"slmp: "+str(self.slmp))
        # elif self.ila == "round":
        #     self.signal_pub.publish("round, "+"slmp: "+str(self.slmp))
        # elif self.ila == "turn":
        #     self.signal_pub.publish("turn, "+"slmp: "+str(self.slmp))
        # elif self.ila == "no speed limit":
        #     self.signal_pub.publish("no speed limit, "+"slmp: "+str(self.slmp))
        # else:
        #     self.signal_pub.publish("No hay matches, "+"slmp: "+str(self.slmp))
        # else:        
        #     self.signal_pub.publish("No detecta nada, "+"slmp: "+str(self.slmp))


    def kalman_filter(self,sample):
        #Create the filter
        self.s_pred = self.a*self.s_pred
        error = sample - self.s_pred 

        self.M = self.a**2*self.M+self.sigma_u          # Calculate prediction of MSE (Mean Square Error)
        K = self.M/(self.sigma_u+self.M)           # Calculate Kalman gain
        self.s_pred = self.s_pred + K*error        # Calculate new estimation with the prediction
        # self.s_hat.append(self.s_pred)
        self.M = (1-K)*self.M                                # Calculate new Mean Square Error
        

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
        rospy.loginfo("Stopping signal detector.")

if __name__ == '__main__':
    signal_detector = Signal_Detector()
    try:
        signal_detector.run()
    except:
        pass


