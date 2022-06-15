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
        self.desired_point_global = [0,0]
        self.p_transformed = [0,0]
        self.v = 0.04    #Linear velocity
        self.omega = 0.0      #Angular velocity
        self.dt = 0.01        

        self.msg_vel = Twist()
        self.desired_angle = 0
        self.dtheta = 0
        self.activate = False

        rospy.init_node('PP_sp')
        rospy.Subscriber('/odom', Pose2D, self.odom_callback)
        rospy.Subscriber('/activator', String, self.activator_callback)
        rospy.Subscriber('/pp_point', Pose2D, self.point_callback)
        self.pp_pub = rospy.Publisher('/pp_point', Pose2D, queue_size=10)
        self.move_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.t1 = rospy.Timer(rospy.Duration(self.dt), self.timer_callback)
        self.rate = rospy.Rate(1/self.dt)
        
        rospy.on_shutdown(self.stop)
    
    def activator_callback(self,msg):
        if msg.data == "PP_activate":
            self.activate = True
        elif msg.data == "PP_deactivate":
            self.activate = False

    def odom_callback(self,msg): #Determine the current location and direction
        self.X = msg.x
        self.Y = msg.y
        self.theta = msg.theta

    def point_callback(self, msg):
        if msg.x == msg.y == msg.theta == 100: return
        self.desired_point_global = [msg.x, msg.y]
        self.desired_point_robot = self.tp_global_to_robot(msg.x, msg.y)
        self.desired_angle = msg.theta
        self.omega = self.v * 2*self.desired_point_robot[1] / (self.desired_point_robot[0]**2 + self.desired_point_robot[1]**2)
        self.msg_vel.linear.x = self.v
        self.msg_vel.angular.z = self.omega

    def tp_robot_to_global(self, x_robot, y_robot):
        R = np.array([[np.cos(self.theta), -np.sin(self.theta)], [np.sin(self.theta), np.cos(self.theta)]])
        d = np.array([[self.X],[self.Y]])
        return np.transpose(R.dot(np.array([[x_robot],[y_robot]])) + d)[0].tolist()

    def tp_global_to_robot(self, x_global, y_global):
        R_t = np.transpose(np.array([[np.cos(self.theta), -np.sin(self.theta)], [np.sin(self.theta), np.cos(self.theta)]]))
        d = np.array([[self.X],[self.Y]])
        return np.transpose(R_t.dot(np.array([[x_global],[y_global]]) - d))[0].tolist()
    
    def timer_callback(self, time):
        if not self.activate:
            return

        if ((self.X - self.desired_point_global[0])**2 + (self.Y - self.desired_point_global[1])**2)**0.5 > 0.3:
            self.move_pub.publish(self.msg_vel)
        #     self.dtheta = self.desired_angle - self.theta
        #     self.dtheta = math.atan2(math.sin(self.dtheta),math.cos(self.dtheta))
        # elif self.dtheta > 0.01:
        #     self.msg_vel.linear.x = 0
        #     kap = 5.5
        #     kad = 1.5
        #     dt = 0.001
        #     dtheta_ant = self.dtheta
        #     self.dtheta = self.desired_angle - self.theta
        #     self.dtheta = math.atan2(math.sin(self.dtheta),math.cos(self.dtheta))
        #     self.omega =  kap*self.dtheta + kad*(self.dtheta - dtheta_ant)/dt
        #     self.msg_vel.angular.z = self.omega
        #     self.move_pub.publish(self.msg_vel)
        
        else:
            msg = Pose2D() 
            msg.x = 100
            msg.y = 100
            msg.theta = 100
            self.pp_pub.publish(msg)

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

