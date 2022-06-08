#!/usr/bin/env python


import rospy
import numpy as np
from geometry_msgs.msg import Pose2D
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import math

class Pure_pursuit:

    def __init__(self):
        self.X = 0.0
        self.Y = 0.0
        self.theta = 0.0
        
        self.v = 0.12      #Linear velocity
        self.omega = 0.0      #Angular velocity
        self.dt = 0.01        

        self.goal = False
        self.msg_vel = Twist()
        self.desired_angle = 0
        self.dtheta = 0

        rospy.init_node('PP_sp')
        rospy.Subscriber('/odom', Pose2D, self.odom_callback)
        rospy.Subscriber('/activator', String, self.activator_callback)
        rospy.Subscriber('/PP_point', Pose2D, self.point_callback)
        self.move_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        self.t1 = rospy.Timer(rospy.Duration(self.dt), self.timer_callback)

        self.rate = rospy.Rate(1/self.dt)
        
        rospy.on_shutdown(self.stop)
    
    def activator_callback(self,msg):
        if msg.data == "PP_activate":
            self.activate = True
        elif msg.data == "PP_deactivate":
            self.activate = False
        elif msg.data == "PP_NotGoal":
            self.goal = False
        elif msg.data == "PP_Goal":
            self.goal = True


    def odom_callback(self,msg): #Determine the current location and direction
        self.X = msg.x
        self.Y = msg.y
        self.theta = msg.theta

    def point_callback(self, msg):
        self.desired_angle = msg.theta
        self.omega = self.v * 2*float(msg.y) / (msg.x**2 + msg.y**2)
        self.msg_vel.linear.x = self.v
        self.msg_vel.angular.z = self.omega
        self.move_pub.publish(self.msg_vel)
    
    def timer_callback(self, time):
        if not self.activate:
            return

        if self.goal and dtheta > 0.01:
            self.msg_vel.linear.x = 0
            kap = 5.5
            kad = 1.5
            dt = 0.001
            dtheta_ant = dtheta
            dtheta = self.desired_angle - self.theta
            dtheta = math.atan2(math.sin(dtheta),math.cos(dtheta))
            self.omega =  kap*dtheta + kad*(dtheta - dtheta_ant)/dt
            self.msg_vel.angular.z = self.omega
            self.move_pub.publish(self.msg_vel)
        else:
            self.move_pub.publish(self.msg_vel)
            dtheta = self.desired_angle - self.theta
            dtheta = math.atan2(math.sin(dtheta),math.cos(dtheta))

    def run(self):
        rospy.spin()

    def stop(self):
        rospy.loginfo("Stopping pure pursuit")

if __name__ == '__main__':
    pp = Pure_pursuit()

    try:
        pp.run()
    except:
        pass

