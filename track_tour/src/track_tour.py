#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import String
from std_msgs.msg import Bool
from geometry_msgs.msg import Pose2D
import numpy as np

rep = 5
class Track_tour:
    def __init__(self):
        #Track tour
        self.dt = 0.1
        self.linear_speed = 0.04  
        self.angular_speed = 0.0
        self.activate_msg = String()
        self.msg_vel = Twist()
        self.at_intersection = False
        self.good_orientation = None

        #Line detection
        self.line = [640,640,640,640]
        self.error = 0.0
        self.x_center = 640

        #Traffic light detection
        self.pp_msg = Pose2D()
        self.num_intersection = 0
        self.con_flag = True

        self.X = 0
        self.Y = 0
        self.theta = 0

        rospy.init_node('track_tour')
        rospy.Subscriber('/odom', Pose2D, self.odom_callback)
        rospy.Subscriber('/line_detector/line', Float32MultiArray, self.line_callback)
        rospy.Subscriber('/line_detector/check', Bool, self.intersection_callback)
        rospy.Subscriber('/traffic_light/first/detection', String, self.tl0_callback)
        rospy.Subscriber('/traffic_light/second/detection', String, self.tl1_callback)
        rospy.Subscriber('/signal_detection', String, self.signal_callback)
        rospy.Subscriber('/pure_pursuit/check', Bool, self.pp_check_callback)

        self.move_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.activator_pub = rospy.Publisher('/activator', String, queue_size=10)
        self.pp_pub = rospy.Publisher('/pure_pursuit/point', Pose2D, queue_size=10)
        self.rate = rospy.Rate(1/self.dt)
        self.timer = rospy.Timer(rospy.Duration(self.dt), self.timer_callback)
        # self.send_activator("OD_activate")
        # self.send_activator("LD_activate")
        rospy.on_shutdown(self.stop)

    def tp_robot_to_global(self, x_robot, y_robot):
        R = np.array([[np.cos(self.theta), -np.sin(self.theta)], [np.sin(self.theta), np.cos(self.theta)]])
        d = np.array([[self.X],[self.Y]])
        return np.transpose(R.dot(np.array([[x_robot],[y_robot]])) + d)[0].tolist()
    
    def send_activator(self, msg):
        self.activate_msg.data = msg
        self.activator_pub.publish(self.activate_msg)
    
    def timer_callback(self, time):
        if self.activate_msg.data != "PP_activate":
            self.move_pub.publish(self.msg_vel)
        else:
            self.pp_pub.publish(self.pp_msg)
        # self.activator_pub.publish(self.activate_msg.data)

    def odom_callback(self, msg): #Determine the current location and direction
        self.X = msg.x
        self.Y = msg.y
        self.theta = msg.theta

    def pp_check_callback(self, msg):
        if msg.data:
            # for _ in range(rep):
            self.send_activator("LD_activate")

    def signal_callback(self, msg):
        # if self.activate_msg != "PP_activate" and self.msg_vel.linear.x == 0 and self.msg_vel.angular.z == 0:
        if self.at_intersection:
            if msg.data == "continue" and self.con_flag:
                self.con_flag = False
                self.turn_flag = True
                rospy.loginfo("Signal: "+msg.data)
                self.num_intersection = 0
                
                if self.good_orientation is not None:
                    angle = self.good_orientation - self.theta
                    R = np.array([[np.cos(angle), -np.sin(angle)], [np.sin(angle), np.cos(angle)]])
                    point = np.transpose(R.dot(np.array([[0.6],[0.0]])))[0].toList()
                    point = self.tp_robot_to_global(point[0],point[1])
                    
                else:
                    try:
                        m = (self.line[3] - self.line[1])/(self.line[2] - self.line[0])
                        x_proyection = -self.line[1]/m + self.line[0]
                    except:
                        x_proyection = self.x_center
                    error_pixeles = self.x_center - x_proyection
                    error_cm = error_pixeles * 0.06/100
                    point = self.tp_robot_to_global(0.6,error_cm)

                self.pp_msg.x = point[0]
                self.pp_msg.y = point[1]
                self.pp_pub.publish(self.pp_msg)
                self.send_activator("TL_activate")
                self.at_intersection = False

            elif msg.data == "turn" and self.turn_flag:
                self.turn_flag = False
                self.con_flag = True
                rospy.loginfo("Signal: "+msg.data)
                self.num_intersection = 1
                try:
                    m = (self.line[3] - self.line[1])/(self.line[2] - self.line[0])
                    x_proyection = (360-self.line[1])/m + self.line[0]
                except:
                    x_proyection = self.x_center
                error_pixeles = self.x_center - x_proyection
                error_cm = error_pixeles * 0.06/100
                point = self.tp_robot_to_global(0.4,error_cm/2-0.25)
                self.pp_msg.x = point[0]
                self.pp_msg.y = point[1]
                self.pp_pub.publish(self.pp_msg)
                self.send_activator("TL_activate")
                self.at_intersection = False

        # elif self.msg_vel.linear.x != 0 and self.msg_vel.angular.z != 0:
        else:
            if msg.data == "no speed limit" and self.spl_flag:
                self.spl_flag = False
                self.linear_speed = 0.12
                self.stop_flag = True
                rospy.loginfo("Signal: "+msg.data)
            elif msg.data == "stop" and self.stop_flag:
                self.stop_flag = False
                self.msg_vel.linear.x = 0
                self.msg_vel.angular.z = 0
                self.spl_flag = True
                rospy.loginfo("Signal: "+msg.data)
        # if msg.data == "stop":
        #     pass
        # elif msg.data == "continue":
        #     self.num_intersection = 0
        # elif msg.data == "round":
        #     pass
        # elif msg.data == "turn":
        #     self.num_intersection = 1
        # elif msg.data == "no speed limit":
        #     self.linear_speed = 0.12
        # elif msg.data == "no":
        #     self.num_intersection = -1

    def tl0_callback(self, msg):
        if msg.data == "green" and self.num_intersection == 0:
            # for _ in range(rep):
            self.send_activator("TL_deactivate")
            self.send_activator("PP_activate")
            self.num_intersection = -1

    def tl1_callback(self, msg):
        if msg.data == "green" and self.num_intersection == 1:
            # for _ in range(rep):
            self.send_activator("TL_deactivate")
            self.send_activator("PP_activate")
            self.num_intersection = -1

    def line_callback(self, msg):
        self.line = msg.data
        if msg.data[0] == msg.data[2] and self.good_orientation is None:
            self.good_orientation = self.theta
        kp = 0.005 * self.linear_speed*0.7 
        kd = 0.0003 * self.linear_speed
        x_b = self.line[0] if self.line[1] <= self.line[3] else self.line[2]
        error_ant = self.error
        self.error = self.x_center - x_b
        self.angular_speed = kp*self.error + kd*(self.error - error_ant)/self.dt
        self.msg_vel.linear.x = self.linear_speed
        self.msg_vel.angular.z = self.angular_speed

    def intersection_callback(self, msg):
        if msg.data:
            self.msg_vel.linear.x = 0
            self.msg_vel.angular.z = 0
            self.at_intersection = True


    def run(self):
        rospy.spin()
    
    def stop(self):
        rospy.loginfo("Stopping track tour.")
        self.timer.shutdown()
        t = Twist()
        t.linear.x = 0.0
        t.angular.z = 0.0
        self.move_pub.publish(t)
        self.move_pub.publish(t)
        self.move_pub.publish(t)

if __name__ == '__main__':
    track_tour = Track_tour()
    try:
        track_tour.run()
    except:
        pass