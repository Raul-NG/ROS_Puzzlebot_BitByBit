#!/usr/bin/env python

from cmath import cos
import rospy
from std_msgs.msg import Float32
from std_msgs.msg import String
from geometry_msgs.msg import Pose2D
import numpy as np

class Odometry:
    def __init__(self):
        self.wr = 0.0
        self.wl = 0.0

        self.xdot = 0.0
        self.thetadot = 0.0

        self.r = 0.05
        self.l = 0.18
        
        self.global_X = 0.0
        self.global_Y = 0.0
        self.theta = 0.0

        self.dt = 0.01
        
        rospy.init_node('odometry')
        rospy.Subscriber('/wr', Float32, self.wr_callback)
        rospy.Subscriber('/wl', Float32, self.wl_callback)
        self.odom_pub = rospy.Publisher('/odom', Pose2D, queue_size=10)

        self.t1 = rospy.Timer(rospy.Duration(self.dt), self.timer_callback)

        self.rate = rospy.Rate(100)
        
        rospy.on_shutdown(self.stop)

    def timer_callback(self, time):
        self.xdot = self.r*(self.wr + self.wl)*0.5
        self.thetadot = self.r * (self.wr - self.wl)/self.l

        self.theta += self.thetadot*self.dt
        self.global_X += self.xdot*np.cos(self.theta)*self.dt
        self.global_Y += self.xdot*np.sin(self.theta)*self.dt

        msg = Pose2D()
        msg.x = self.global_X
        msg.y = self.global_Y
        msg.theta =  self.theta
        self.odom_pub.publish(msg)

    def wr_callback(self, msg):
        self.wr = msg.data

    def wl_callback(self, msg):
        self.wl = msg.data

    def run(self):
        rospy.spin()

    def stop(self):
        print('Killing odom')

if __name__ == '__main__':
    od = Odometry()

    try:
        od.run()
    except:
        pass
