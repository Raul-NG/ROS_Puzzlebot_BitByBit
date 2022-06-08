#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray
import numpy as np

class Line_Detector:
    def __init__(self):
        self.dt = 0.1
        self.error_horizontal = 0.0
        self.error_angle = 0.0
        self.error = 0.0
        self.x_center = 640
        self.linear_speed = 0.12      #Linear velocity
        self.angular_speed = 0.0
        self.max_omega = np.pi/4    #Maximum angular velocity
        self.cut_y = (int(3*720.0/4.0),720)
        self.cut_x = (0,1280)
        # self.cut_x = (320,960)
        self.edges = None

        self.tem = []


        rospy.init_node('Line_Detector')
        rospy.Subscriber('/video_source/raw', Image, self.img_callback)
        # rospy.Subscriber('/odom', Pose2D, self.odom_callback)
        self.edges_pub = rospy.Publisher('/img_properties/edges', Image, queue_size=10)
        self.lines_pub = rospy.Publisher('/img_properties/lines', Image, queue_size=10)
        self.canny_1 = rospy.Publisher('/img_properties/canny_1', Image, queue_size=10)
        self.canny_2 = rospy.Publisher('/img_properties/canny_2', Image, queue_size=10)
        self.move_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.rate = rospy.Rate(1/self.dt)
        self.timer = rospy.Timer(rospy.Duration(self.dt), self.timer_callback)
        rospy.on_shutdown(self.stop)

    def timer_callback(self, time):
        self.lines = []
        for _ in range(2):
            self.detect_lines()
        if len(self.lines) > 70:
            # self.check_trff_lgt()
            t = Twist()
            t.linear.x = 0.0
            t.angular.z = 0.0
            self.move_pub.publish(t)
        else:
            self.choose_line()
            self.navigate()
        self.show_lines()

    def img_callback(self,msg):
        self.image_raw = self.bridge.imgmsg_to_cv2(msg, "passthrough")

    def detect_lines(self):
        # Grayscale and Canny Edges extracted
        gray_image = cv2.cvtColor(self.image_raw, cv2.COLOR_BGR2GRAY)
        self.canny_1.publish(self.bridge.cv2_to_imgmsg(gray_image))
        _,msk = cv2.threshold(gray_image,90,255, cv2.THRESH_BINARY)
        self.canny_2.publish(self.bridge.cv2_to_imgmsg(msk))
        msk = cv2.GaussianBlur(msk, (9,9), cv2.BORDER_DEFAULT)
        erode = cv2.erode(msk, np.ones((5, 5), np.uint8), iterations = 4)
        
        # self.edges = erode
        edges = cv2.Canny(erode, 150, 200)
        
        self.edges = cv2.dilate(edges, np.ones((5, 5), np.uint8), iterations = 1)
        self.edges_pub.publish(self.bridge.cv2_to_imgmsg(self.edges))
        linesP = cv2.HoughLinesP(self.edges[self.cut_y[0]:self.cut_y[1],self.cut_x[0]:self.cut_x[1]], 1, np.pi / 180, 50, None, 100, 80)
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
    
    # def check_trff_lgt(self):

    #     self.orb = cv2.ORB_create(1000)
    #     self.orb.setScaleFactor(1.2)
    #     FLANN_INDEX_KDTREE = 0
    #     index_params = dict(algorithm = FLANN_INDEX_KDTREE, trees = 3)
    #     search_params = dict(checks = 100)
    #     self.flann = cv2.FlannBasedMatcher(index_params, search_params)

    #     dest = []
    #     for i in range(1,6):
    #         _, destemp = self.orb.detectAndCompute(cv2.cvtColor(cv2.imread('semaforo_t_%d.png' % i), cv2.COLOR_BGR2GRAY), None)
    #         dest.append(np.float32(destemp))

    #     colores = ['verde', 'amarillo', 'rojo','apagado 1', 'apagado 2']
    #     cap= cv2.VideoCapture(0)
    #     slml = []
    #     il = []
    #     slma = 0
    #     ila = 0
    #     while True:
    #         ret, frame= cap.read()
    #         if cv2.waitKey(1) & 0xFF == 13 or ret == False:
    #             break
    #         gfr = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    #         kpf, desf = orb.detectAndCompute(gfr, None)
    #         desf = np.float32(desf)
    #         slm = 0
    #         index = 0
    #         matches = []
    #         matchesMask = []
    #         for j in range(5):
    #             matches.append(self.flann.knnMatch(dest[j], desf, k=2))
    #             matchesMask.append([[0,0] for k in range(len(matches[j]))])
    #             for k,(m,n) in enumerate(matches[j]):
    #                 if m.distance < 0.7*n.distance:
    #                     matchesMask[j][k]=[1,0]
    #             matchesMask[j] = np.array(matchesMask[j])
    #             if slm < np.sum(matchesMask[j][:,0]):
    #                 slm = np.sum(matchesMask[j][:,0])
    #                 index = j
    #         slml.append(slm)
    #         il.append(index)
    #         if len(slml) > 10:
    #             slml.pop(0)
    #             il.pop(0)
    #             slma = round(np.median(np.array(slml)))
    #             ila = round(np.median(np.array(il)))
    #         font = cv2.FONT_HERSHEY_SIMPLEX
    #         org = (0,frame.shape[1] - 170)
    #         fontScale = 1
    #         color = (255,255,255)
    #         thickness = 1
    #         text1 = cv2.putText(frame, str(slma), org, font, fontScale, color, thickness, cv2.LINE_AA)
    #         org1 = (0,30)
    #         if slma > 2:
    #             text2 = cv2.putText(text1, colores[ila], org1, font, fontScale, color, thickness, cv2.LINE_AA)
    #         else:
    #             text2 = cv2.putText(text1, "no", org1, font, fontScale, color, thickness, cv2.LINE_AA)
    #         cv2.imshow('Matching', text2)



    #     t = Twist()
    #     t.linear.x = 0.0
    #     t.angular.z = 0.0
    #     self.move_pub.publish(t)


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


