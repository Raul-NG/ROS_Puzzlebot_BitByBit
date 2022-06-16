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
        self.intersection = 0
        self.first_int = True


        rospy.init_node('track_tour')
        rospy.Subscriber('/pure_pursuit/talkback', String, self.pp_talkback_callback)
        rospy.Subscriber('/line_detector/talkback', String, self.ld_talkback_callback)
        self.activator_pub = rospy.Publisher('/activator', String, queue_size=10)
        self.pp_selector = rospy.Publisher('/pure_pursuit/trajectory_selector', Int16, queue_size=10)

        self.t1 = rospy.Timer(rospy.Duration(self.dt), self.timer_callback)

        self.rate = rospy.Rate(100)
        
        rospy.on_shutdown(self.stop)

    def pp_talkback_callback(self,msg):
        if msg.data == "done":
            self.activator_pub.publish("LD_activate")
    
    def ld_talkback_callback(self,msg):
        if msg.data == "intersection":
            self.num_intersection = 0 if self.first_int == True else 1
            self.first_int = False
            if self.num_intersection == 0:
                self.pp_selector.publish(1)
            elif self.num_intersection ==1:
                self.pp_selector.publish(2)
            self.activator_pub.publish("LD_deactivate")
            self.activator_pub.publish("PP_activate")
        
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
