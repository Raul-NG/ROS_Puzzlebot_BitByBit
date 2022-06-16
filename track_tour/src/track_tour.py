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
        self.sign_pub = rospy.Publisher('/signal_detect', String, queue_size=10)
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
            self.pp_finish = True
            self.activator_pub.publish("LD_activate")
            
    def signal_callback(self,msg):
        self.signal = msg.data
        if self.num_intersection == "2" and self.signal == "stop" and self.pp_finish:
            self.sign_pub.publish("stop")
            t = Twist()
            t.linear.x = 0.0
            t.angular.z = 0.0
            self.move_pub.publish(t)
            self.move_pub.publish(t)
            self.move_pub.publish(t)
        if self.num_intersection == "1" and self.signal == "no_speed_limit" and self.pp_finish:
            self.sign_pub.publish("no_speed_limit")
            t = Twist()
            t.linear.x = 0.2
            t.angular.z = 0.0
            self.move_pub.publish(t)
            self.move_pub.publish(t)
            self.move_pub.publish(t)

    def ld_talkback_callback(self,msg):
        # if msg.data == "intersection": 
        self.num_intersection = msg.data
        self.activator_pub.publish("LD_deactivate")
        rospy.loginfo("LD_deactivate")
        if msg.data == "1" and self.signal == "continue":
            self.sign_pub.publish("continue")
            self.pp_selector.publish(1)
            self.activator_pub.publish("TL_activate")
            rospy.loginfo("TL_activate1")
            # self.num_intersection = 1
        elif msg.data == "2" and self.signal == "turn":
            self.sign_pub.publish("turn")
            self.pp_selector.publish(2)
            self.activator_pub.publish("TL_activate")
            rospy.loginfo("TL_activate2")
            # self.num_intersection = 2
        elif msg.data == "3":
            t = Twist()
            t.linear.x = 0.0
            t.angular.z = 0.0
            self.move_pub.publish(t)
            self.move_pub.publish(t)
            self.move_pub.publish(t)

    def tl_callback(self,msg):
        self.traffic_light = msg.data 
        if self.traffic_light == "green":
            rospy.loginfo("Semaforo green")
            self.activator_pub.publish("TL_deactivate")
            self.pp_finish = False
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
