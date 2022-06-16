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
        self.traffic_light = ""
        self.signal = ""


        rospy.init_node('Track_tour')
        rospy.Subscriber('/pure_pursuit/talkback', String, self.pp_talkback_callback)
        rospy.Subscriber('/line_detector/talkback', String, self.ld_talkback_callback)
        rospy.Subscriber('/traffic_light/detection', String, self.tl_callback)
        rospy.Subscriber('/signal_detection', String, self.signal_callback)
        self.activator_pub = rospy.Publisher('/activator', String, queue_size=10)
        self.pp_selector = rospy.Publisher('/pure_pursuit/trajectory_selector', Int16, queue_size=10)
        self.move_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        
        
        self.activator_pub.publish("LD_activate")
        self.activator_pub.publish("PP_deactivate")
        self.activator_pub.publish("TL_deactivate")
        

        self.t1 = rospy.Timer(rospy.Duration(self.dt), self.timer_callback)

        self.rate = rospy.Rate(100)
        
        rospy.on_shutdown(self.stop)

    def timer_callback(self,time):
        pass

    def pp_talkback_callback(self,msg):
        if msg.data == "done":
            self.activator_pub.publish("LD_activate")
            self.intersection_flag = False
            
    def signal_callback(self,msg):
        self.signal = msg.data
        if self.intersection_flag:
            if msg.data == "continue":
                self.pp_selector.publish(1)
                self.activator_pub.publish("TL_activate")
                rospy.loginfo("TL_activate1")
            elif msg.data == "turn":
                self.pp_selector.publish(2)
                self.activator_pub.publish("TL_activate")
        else:
            if msg.data == "no speed limit":
                self.activator_pub.publish("No speed limit")
            elif msg.data == "stop":
                t = Twist()
                t.linear.x = 0.0
                t.angular.z = 0.0
                self.move_pub.publish(t)
                self.move_pub.publish(t)
                self.move_pub.publish(t)

    def ld_talkback_callback(self,msg):
        # if msg.data == "intersection": 
        # if self.num_intersection == 0:
        self.intersection_flag = True
        self.activator_pub.publish("LD_deactivate")
        rospy.loginfo("LD_deactivate")
        # if msg.data == "1":
        #     self.pp_selector.publish(1)
        #     self.activator_pub.publish("TL_activate")
        #     rospy.loginfo("TL_activate1")
        #     # self.num_intersection = 1
        # elif msg.data == "2":
        #     self.pp_selector.publish(2)
        #     self.activator_pub.publish("TL_activate")
        #     rospy.loginfo("TL_activate2")
        #     # self.num_intersection = 2
        # elif msg.data == "3":
        #     t = Twist()
        #     t.linear.x = 0.0
        #     t.angular.z = 0.0
        #     self.move_pub.publish(t)
        #     self.move_pub.publish(t)
        #     self.move_pub.publish(t)

    def tl_callback(self,msg):
        self.traffic_light = msg.data 
        if self.traffic_light == "green":
            rospy.loginfo("Semaforo green")
            self.activator_pub.publish("TL_deactivate")
            self.activator_pub.publish("PP_activate")
        else:
            rospy.loginfo("Semaforo red")
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
