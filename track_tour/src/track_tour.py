#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import String
from std_msgs.msg import Int16
from geometry_msgs.msg import Pose2D
import numpy as np

class Track_tour:

    def __init__(self):
        self.dt = 0.1
        self.error = 0.0
        self.intersection = -1
        self.first_int = True
        self.num_intersection = 0


        rospy.init_node('Track_tour')
        rospy.Subscriber('/pure_pursuit/talkback', String, self.pp_talkback_callback)
        rospy.Subscriber('/line_detector/talkback', String, self.ld_talkback_callback)
        self.activator_pub = rospy.Publisher('/activator', String, queue_size=10)
        self.pp_selector = rospy.Publisher('/pure_pursuit/trajectory_selector', Int16, queue_size=10)
        self.move_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        
        self.activator_pub.publish("LD_activate")
        self.activator_pub.publish("PP_deactivate")
        

        self.t1 = rospy.Timer(rospy.Duration(self.dt), self.timer_callback)

        self.rate = rospy.Rate(100)
        
        rospy.on_shutdown(self.stop)

    def timer_callback(self,time):
        pass

    def pp_talkback_callback(self,msg):
        if msg.data == "done":
            self.activator_pub.publish("LD_activate")
    
    def ld_talkback_callback(self,msg):
        # if msg.data == "intersection": 
        # if self.num_intersection == 0:
        if msg.data == "1":
            self.pp_selector.publish(1)
            self.activator_pub.publish("LD_deactivate")
            self.activator_pub.publish("PP_activate")
            # self.num_intersection = 1
        elif msg.data == "2":
            self.pp_selector.publish(2)
            self.activator_pub.publish("LD_deactivate")
            self.activator_pub.publish("PP_activate")
            # self.num_intersection = 2
        elif msg.data == "3":
            self.activator_pub.publish("LD_deactivate")
            t = Twist()
            t.linear.x = 0.0
            t.angular.z = 0.0
            self.move_pub.publish(t)
            self.move_pub.publish(t)
            self.move_pub.publish(t)


            
        
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
