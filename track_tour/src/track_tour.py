#!/usr/bin/env python

from multiprocessing.dummy import active_children
from signal import signal
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import String
from geometry_msgs.msg import Pose2D
import numpy as np

class Track_tour:
    def __init__(self):
        self.dt = 0.1
        self.error = 0.0
        self.x_center = 640
        self.linear_speed = 0.04  
        self.angular_speed = 0.0
        self.activate_msg = String()
        self.activate_ant = ""
        self.line = [640,640,640,640]
        self.odometry = Pose2D()
        self.pp_msg = Pose2D()
        self.msg_vel = Twist()
        self.pp_turn = 0
        self.num_intersection = 0

        self.X = 0
        self.Y = 0
        self.theta = 0

        rospy.init_node('track_tour')
        rospy.Subscriber('/line_detector', Float32MultiArray, self.line_callback)
        rospy.Subscriber('/odom', Pose2D, self.odom_callback)
        rospy.Subscriber('/traffic_light', String, self.tl_callback)
        rospy.Subscriber('/pp_point', Pose2D, self.pp_callback)
        rospy.Subscriber('/signal', String, self.signal_callback)
        self.move_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.activator_pub = rospy.Publisher('/activator', String, queue_size=10)
        self.pp_pub = rospy.Publisher('/pp_point', Pose2D, queue_size=10)
        self.rate = rospy.Rate(1/self.dt)
        self.timer = rospy.Timer(rospy.Duration(self.dt), self.timer_callback)

        # self.activate_msg.data = "OD_activate"
        # self.activator_pub.publish(self.activate_msg)
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

    def odom_callback(self, msg): #Determine the current location and direction
        self.X = msg.x
        self.Y = msg.y
        self.theta = msg.theta

    def pp_callback(self, msg):
        if msg.x == msg.y == msg.theta == 100:
            for _ in range(10):
                self.activate_msg.data = "PP_deactivate" #PP, LD, TL
                self.activator_pub.publish(self.activate_msg)
            for _ in range(10):
                self.activate_msg.data = "LD_activate" #PP, LD, TL
                self.activator_pub.publish(self.activate_msg)
            # self.msg_vel.linear.x = 0
            # self.msg_vel.angular.z = 0

    def signal_callback(self, msg):
        if msg.data == "stop":
            pass
        elif msg.data == "continue":
            pass
        elif msg.data == "round":
            pass
        elif msg.data == "turn":
            pass
        elif msg.data == "no speed limit":
            self.linear_speed = 0.12
        elif msg.data == "No hay matches":
            pass
    
    def tl_callback(self, msg):
        if msg.data[0] != self.num_intersection:
            return
        
        if msg.data[1:] == "green":
            for _ in range(10):
                self.activate_msg.data = "TL_deactivate" #PP, LD, TL
                self.activator_pub.publish(self.activate_msg)
                # punto 1 o punto 2
            for _ in range(10):
                self.activate_msg.data = "PP_activate"
                self.activator_pub.publish(self.activate_msg)

            #functionPP(action)
        else:
            self.msg_vel.linear.x = 0
            self.msg_vel.angular.z = 0
        """
        if msg.data == "red":
            self.msg_vel.linear.x = 0
            self.msg_vel.angular.z = 0
        else:
            self.activate_msg.data = "TL_deactivate" #PP, LD, TL
            self.activator_pub.publish(self.activate_msg)
            # if self.pp_turn == 0:
            #     x = 0.4
            #     y = -(self.line[2]-self.line[0])/(self.line[3]-self.line[1])()
            #     # point = [1.0, 0.0]
            #     self.pp_msg.theta = 0.0
            # #     self.pp_turn = 1
            # # elif self.pp_turn == 1:
            # #     x = 0.30
            # #     y = -0.20
            # #     self.pp_msg.theta = -np.pi/2
            # #     self.pp_turn = 0
            # point = self.tp_robot_to_global(x,y)
            # self.pp_msg.x = point[0]
            # self.pp_msg.y = point[1]
            # self.pp_pub.publish(self.pp_msg)
            self.activate_msg.data = "PP_activate"
            self.activator_pub.publish(self.activate_msg)
        """

    def line_callback(self, msg):
        if msg.data[0] < 0:
            # self.num_intersection = 0 if self.Y <= 0.2 else 1
            try:
                m = (self.line[3] - self.line[1])/(self.line[2] - self.line[0])
                x_proyection = (self.num_intersection*360-self.line[1])/m + self.line[0]
            except:
                x_proyection = self.x_center
            error_pixeles = self.x_center - x_proyection
            error_cm = error_pixeles * 0.065/100

            point = self.tp_robot_to_global(0.6,error_cm) if self.num_intersection == 0 else self.tp_robot_to_global(0.4,error_cm/2-0.25)
            self.pp_msg.x = point[0]
            self.pp_msg.y = point[1]
            rospy.loginfo("intersection: "+str(self.num_intersection))
            self.num_intersection += 1
            self.pp_pub.publish(self.pp_msg)
            for _ in range(10):
                self.activate_msg.data = "LD_deactivate"
                self.activator_pub.publish(self.activate_msg)
            for _ in range(10):
                self.activate_msg.data = "TL_activate"
                self.activator_pub.publish(self.activate_msg)
        else:
            self.line = msg.data
            kp = 0.005 * self.linear_speed
            kd = 0.0003 * self.linear_speed
            x_b = self.line[0] if self.line[1] <= self.line[3] else self.line[2]
            error_ant = self.error
            self.error = self.x_center - x_b
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
        self.move_pub.publish(t)
        self.move_pub.publish(t)

if __name__ == '__main__':
    track_tour = Track_tour()
    try:
        track_tour.run()
    except:
        pass