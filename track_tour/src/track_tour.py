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
        self.lines= Float32MultiArray()
        self.odometry = Pose2D()
        self.pp_msg = Pose2D()
        
        self.pp_turn = 0

        self.X = 0
        self.Y = 0
        self.theta = 0
        self.pos = np.array([[0],[0]]) 




        rospy.init_node('Track_navigator')
        rospy.Subscriber('/Line_detector', Float32MultiArray, self.line_callback)
        rospy.Subscriber('/activator', String, self.activator_callback)
        rospy.Suscriber('/odom', Pose2D, self.odom_callback)

        self.move_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.activator = rospy.Publisher('/activator', String, queue_size=10)
        self.pp_pub = rospy.Publisher('/pp_point', Pose2D, queue_size=10)
        self.rate = rospy.Rate(1/self.dt)
        self.timer = rospy.Timer(rospy.Duration(self.dt), self.timer_callback)
        self.activate_msg.data = "OD_activate"
        self.activator.publish(self.activate_msg)
        rospy.on_shutdown(self.stop)

    def tp_robot_to_global(self, x_robot, y_robot):
        R = np.array([[np.cos(self.theta), -np.sin(self.theta)], [np.sin(self.theta), np.cos(self.theta)]])
        d = np.array([[self.X],[self.Y]])
        return np.transpose(R.dot(np.array([[x_robot],[y_robot]])) + d)[0].tolist()

    def timer_callback(self, time):
        self.activate_msg.data = "LD_activate" #PP, LD, TL
        self.activator.publish(self.activate_msg)
        if self.lines[0]< 0:
            t = Twist()
            t.linear.x = 0.0
            t.angular.z = 0.0
            self.move_pub.publish(t)

            self.activate_msg.data = "TL_activate" #PP, LD, TL
            self.activator.publish(self.activate_msg)
            self.activate_msg.data = "LD_deactivate" #PP, LD, TL
            self.activator.publish(self.activate_msg)
            self.activate_msg.data = "PP_activate" #PP, LD, TL
            self.activator.publish(self.activate_msg)
            if self.sem == "Rojo":
                return
            else:
                if self.pp_turn == 0:
                    self.pp_msg.x = 0.50 
                    self.pp_msg.y = 0.0
                    self.pp_msg.theta = 0.0
                    self.pp_turn=1
                elif self.pp_turn ==1:
                    self.pp_msg.x = 0.30
                    self.pp_msg.y = -0.20
                    self.pp_msg.theta = -np.pi/2
                    self.pp_turn=0
                self.pp_pub.publish(self.pp_msg)
            p_transformed = self.tp_robot_to_global(self.pp_msg.x, self.pp_msg.y)
            while ((self.X - p_transformed[0])**2 +(self.Y != p_transformed[1])**2)**0.5 > 0.1:
                self.activate_msg.data = "PP_NotGoal"
                self.activator.publish(self.activate_msg)
            while abs(self.pp_msg.theta - self.theta)>0.01:
                self.activate_msg.data = "PP_Goal"
                self.activator.publish(self.activate_msg)
            self.activate_msg.data = "TL_deactivate" #PP, LD, TL
            self.activator.publish(self.activate_msg)
            self.activate_msg.data = "LD_activate" #PP, LD, TL
            self.activator.publish(self.activate_msg)
            self.activate_msg.data = "PP_deactivate" #PP, LD, TL
            self.activator.publish(self.activate_msg)
        else:
            self.navigate()

    def odom_callback(self,msg): #Determine the current location and direction
        self.X = msg.x
        self.Y = msg.y
        self.theta = msg.theta
        np.append(self.pos, [[msg.x], [msg.y]], axis=1)



        
            

    def line_callback(self,msg):
        self.lines = msg.data

    def activator_callback(self,msg):
        self.activate_msg = msg.data

    def navigate(self):
        # kp = 0.05 * self.linear_speed
        # kd = 0.003 * self.linear_speed
        kp = 0.005 * self.linear_speed
        kd = 0.0003 * self.linear_speed
        x = self.line[0] if self.line[1] <= self.line[3] else self.line[2]
        error_ant = self.error
        self.error_horizontal = (self.x_center - x) #/ 640 * 3
        self.error = self.error_horizontal
        self.angular_speed = kp*self.error + kd*(self.error - error_ant)/self.dt

        msg = Twist()
        msg.linear.x = self.linear_speed
        msg.angular.z = self.angular_speed
        self.move_pub.publish(msg)
    

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
    track_tour = Track_tour()
    try:
        track_tour.run()
    except:
        pass


