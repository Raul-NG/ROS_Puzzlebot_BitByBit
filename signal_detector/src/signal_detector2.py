#!/usr/bin/env python 

import rospy 
from std_msgs.msg import String 
from sensor_msgs.msg import Image
from cv_bridge import CvBridge 
import numpy as np 
import cv2

class Signal_Detector: 
    def __init__(self): 
        self.activate = True
        self.time_sleep = True
        self.bridge = CvBridge() 
        self.image_raw = None 
        self.dt = 0.05
        self.error_count = 0
        self.signals = ['continue','turn','round','no_speed_limit','stop','no matches'] 

        self.tem = []
        self.tem_canny = [] 
        self.tml = 5 
        self.orb = cv2.ORB_create(45) 
        # self.orb2 = cv2.ORB_create(1000) 
        self.flann = cv2.FlannBasedMatcher(dict(algorithm = 0, trees = 3, multi_probe_level = 2), dict(checks = 100)) 
        # dict(algorithm = 6, table_number = 12, key_size = 12, multi_probe_level = 2)
        # dict(algorithm = 0, trees = 3)
        self.dest = [] 
        for i in range(self.tml):
            self.tem.append(cv2.imread('/home/puzzlebot/catkin_ws/src/signal_detector/src/Signal_%d.jpeg' % i))
            self.tem[i] = np.pad(cv2.pyrDown(self.tem[i]), pad_width=[(50, 50),(50, 50),(0, 0)], mode='constant',constant_values=(255))

            gray_image = cv2.cvtColor(self.tem[i] , cv2.COLOR_BGR2GRAY)
            _,msk = cv2.threshold(gray_image,150,255, cv2.THRESH_BINARY)
            # msk = cv2.GaussianBlur(msk, (9,9), cv2.BORDER_DEFAULT)
            # msk = cv2.dilate(msk, np.ones((2, 2), np.uint8), iterations = 1)
            # msk = cv2.erode(msk, np.ones((2, 2), np.uint8), iterations = 1)

            # msk = cv2.Canny(msk, 70, 100)
            # msk = cv2.dilate(msk, np.ones((5, 5), np.uint8), iterations = 1)
            self.tem_canny.append(msk)
            
            _, destemp = self.orb.detectAndCompute(self.tem_canny[i], None) 
            self.dest.append(np.float32(destemp)) 
        self.slml = [] 
        self.il = [] 
        self.ila = "" 
        self.slmp = 0 
        self.sizel = 15
        self.last = 'no matches'

        rospy.init_node('signal_detector') 
        rospy.Subscriber('/video_source/raw', Image, self.img_callback)
        self.signal_pub = rospy.Publisher('/signal_detection', String, queue_size=10)
        self.templates_pubs = [rospy.Publisher('/signal_detection/tem/'+tem, Image, queue_size=10) for tem in self.signals[:5]]
        self.image_canny_pub = rospy.Publisher('/signal_detection/image_canny', Image, queue_size=10)
        self.rate = rospy.Rate(1/self.dt)
        self.timer = rospy.Timer(rospy.Duration(self.dt), self.timer_callback)
        rospy.on_shutdown(self.stop) 

    def timer_callback(self, time): 
        self.time_sleep = True
        for num, canny in enumerate(self.tem_canny):
            self.templates_pubs[num].publish(self.bridge.cv2_to_imgmsg(canny))

    def img_callback(self,msg): 
        image = self.bridge.imgmsg_to_cv2(msg, "passthrough")
        gray_image = cv2.cvtColor(image , cv2.COLOR_BGR2GRAY)
        _,msk = cv2.threshold(gray_image,100,255, cv2.THRESH_BINARY)
        # msk = cv2.GaussianBlur(msk, (5,5), cv2.BORDER_DEFAULT)
        # msk = cv2.erode(msk, np.ones((3, 3), np.uint8), iterations = 2)
        # msk = cv2.Canny(msk, 150, 200)
        # msk = cv2.dilate(msk, np.ones((3, 3), np.uint8), iterations = 1)
        self.image_canny_pub.publish(self.bridge.cv2_to_imgmsg(msk))
        self.image_raw = msk

    def run(self):
        while True:
            if self.time_sleep:
                self.time_sleep = False
                _, desf = self.orb.detectAndCompute(self.image_raw, None) 
                desf = np.array(np.float32(desf)) 
                slm = 0 
                index = -1 
                matches = [] 
                matchesMask = [] 
                for j in range(self.tml): 
                    try:
                        matches.append(self.flann.knnMatch(self.dest[j], desf, k=2)) 
                        matchesMask.append([[0,0] for k in range(len(matches[j]))]) 
                        for k,(m,n) in enumerate(matches[j]): 
                            if m.distance < 0.7*n.distance: 
                                matchesMask[j][k]=[1,0] 
                        matchesMask[j] = np.array(matchesMask[j]) 
                        if slm < np.sum(matchesMask[j][:,0]): 
                            slm = np.sum(matchesMask[j][:,0]) 
                            index = j 
                    except:
                        rospy.loginfo("Error: "+str(self.error_count))
                        self.error_count += 1
                if slm == 0: # Possible bug
                    index = -1 
                # self.signal_pub.publish(self.signals[index]+", "+"slmp: "+str(self.slmp)) 
                self.slml.append(slm) 
                self.il.append(index) 
                if len(self.slml) > self.sizel: 
                    self.slml.pop(0)
                    self.il.pop(0)
                self.slmp = np.mean(np.array(self.slml)) 
                vals, counts = np.unique(self.il, return_counts=True) 
                self.ilmo = int(vals[np.argwhere(counts == np.max(counts))][0])
                self.ila = self.signals[self.ilmo] 
                self.ilma = round(np.median(np.array(self.il))) 
                
                if (self.ila != "turn" or self.slmp >= 2.0) and self.last == self.ila:
                    self.signal_pub.publish(self.ila+", "+"slmp: "+str(self.slmp))
                elif self.last != self.ila:
                    self.signal_pub.publish(self.last+", "+"slmp: "+str(self.slmp))
                # if (self.ila != "turn" or self.slmp >= 2.0) and self.last == self.ila and self.ilma == self.ilmo and np.max(counts) > 13:  
                #     self.signal_pub.publish(self.ila+", "+"slmp: "+str(self.slmp)) 
                else:
                    self.signal_pub.publish("No hay matches, "+"slmp: "+str(self.slmp))
                
                self.last = self.ila
    
    def stop(self): 
        rospy.loginfo("Stopping signal detector.") 

if __name__ == '__main__':
    sign_detect = Signal_Detector()
    try:
        sign_detect.run()
    except:
        pass