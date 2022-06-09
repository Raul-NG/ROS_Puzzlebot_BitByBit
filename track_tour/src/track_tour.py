#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import String
from geometry_msgs.msg import Pose2D
import numpy as np

class Track_tour:
    def __init__(self):
        self.dt = 0.1
        self.error_horizontal = 0.0
        self.error = 0.0
        self.x_center = 640
        self.linear_speed = 0.12      #Linear velocity
        self.angular_speed = 0.0
        self.activate_msg = String()
        self.line = [640,640,640,640]
        self.odometry = Pose2D()
        self.pp_msg = Pose2D()
        self.msg_vel = Twist()
        self.pp_turn = 0

        self.X = 0
        self.Y = 0
        self.theta = 0

        rospy.init_node('Track_navigator')
        rospy.Subscriber('/line_detector', Float32MultiArray, self.line_callback)
        rospy.Subscriber('/odom', Pose2D, self.odom_callback)
        rospy.Subscriber('/traffic_light', String, self.tl_callback)
        rospy.Subscriber('/pp_point', Pose2D, self.pp_callback)
        self.move_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.activator_pub = rospy.Publisher('/activator', String, queue_size=10)
        self.pp_pub = rospy.Publisher('/pp_point', Pose2D, queue_size=10)
        self.rate = rospy.Rate(1/self.dt)
        self.timer = rospy.Timer(rospy.Duration(self.dt), self.timer_callback)

        self.activate_msg.data = "OD_activate"
        self.activator_pub.publish(self.activate_msg)
        self.activate_msg.data = "LD_activate" #PP, LD, TL
        self.activator_pub.publish(self.activate_msg)
        rospy.on_shutdown(self.stop)

    def tp_robot_to_global(self, x_robot, y_robot):
        R = np.array([[np.cos(self.theta), -np.sin(self.theta)], [np.sin(self.theta), np.cos(self.theta)]])
        d = np.array([[self.X],[self.Y]])
        return np.transpose(R.dot(np.array([[x_robot],[y_robot]])) + d)[0].tolist()

    def timer_callback(self, time):
        if self.activate_msg.data != "PP_activate":
            self.move_pub.publish(self.msg_vel)
        else:
            self.pp_pub.publish(self.pp_msg)
        self.activator_pub.publish(self.activate_msg.data)

    def odom_callback(self,msg): #Determine the current location and direction
        self.X = msg.x
        self.Y = msg.y
        self.theta = msg.theta

    def pp_callback(self,msg):
        if msg.x == msg.y == msg.theta == 100:
            self.activate_msg.data = "PP_deactivate" #PP, LD, TL
            self.activator_pub.publish(self.activate_msg)
            self.activate_msg.data = "LD_activate" #PP, LD, TL
            self.activator_pub.publish(self.activate_msg)

    def tl_callback(self,msg):
        if msg.data == "Rojo":
            self.msg_vel.linear.x = 0
            self.msg_vel.angular.z = 0
        else:
            self.activate_msg.data = "TL_deactivate" #PP, LD, TL
            self.activator_pub.publish(self.activate_msg)
            if self.pp_turn == 0:
                x = 0.65 
                y = 0.0
                # point = [1.0, 0.0]
                self.pp_msg.theta = 0.0
                self.pp_turn = 1
            elif self.pp_turn == 1:
                x = 0.30
                y = -0.20
                self.pp_msg.theta = -np.pi/2
                self.pp_turn = 0
            point = self.tp_robot_to_global(x,y)
            self.pp_msg.x = point[0]
            self.pp_msg.y = point[1]
            self.pp_pub.publish(self.pp_msg)
            self.activate_msg.data = "PP_activate"
            self.activator_pub.publish(self.activate_msg)

    def line_callback(self,msg):
        self.line = msg.data
        if self.line[0] < 0:
            self.activate_msg.data = "LD_deactivate"
            self.activator_pub.publish(self.activate_msg)
            self.activate_msg.data = "TL_activate"
            self.activator_pub.publish(self.activate_msg)
        else:
            kp = 0.005 * self.linear_speed
            kd = 0.0003 * self.linear_speed
            x = self.line[0] if self.line[1] <= self.line[3] else self.line[2]
            error_ant = self.error
            self.error_horizontal = (self.x_center - x)
            self.error = self.error_horizontal
            self.angular_speed = kp*self.error + kd*(self.error - error_ant)/self.dt
            self.msg_vel.linear.x = self.linear_speed
            self.msg_vel.angular.z = self.angular_speed

    def run(self):
        rospy.spin()
    
    def stop(self):
        rospy.loginfo("Stopping track tour.")
        self.timer.shutdown()
        t = Twist()
        t.linear.x = 0.0
        t.angular.z = 0.0
        self.move_pub.publish(t)

if __name__ == '__main__':
    track_tour = Track_tour()
    try:
        track_tour.run()
    except:
        pass


