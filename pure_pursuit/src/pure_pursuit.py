#!/usr/bin/env python

import rospy
import numpy as np
from geometry_msgs.msg import Pose2D
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from std_msgs.msg import Int16

class Pure_pursuit:

    def __init__(self):
        self.X = 0.0
        self.Y = 0.0
        self.theta = 0.0
        self.odom = Pose2D()
        self.actual_wp = 0 #Index for the closest waypoint
        self.next_wp = 0
        self.goal_transformed = np.array([[],[]])
        self.activate_flag = False
        

        #Pista
        # self.wp = np.array([np.linspace(0,1.8,20),np.linspace(0,0,20)])
        # self.wp = np.append(self.wp, np.array([np.linspace(1.9,1.9,30),np.linspace(0,1.2,30)]),axis=1)
        # self.wp = np.append(self.wp, np.array([np.linspace(1.9,0.8,30),np.linspace(1.2,1.2,30)]),axis=1)
        # self.wp = np.append(self.wp, np.array([np.linspace(0.8,0.8,30),np.linspace(1.2,0.35,30)]),axis=1)


        self.v = 0.12      #Linear velocity
        self.omega = 0.0      #Angular velocity
        self.pos = np.array([[0],[0]])       #Position array

        self.max_omega = 1    #Maximum angular velocity
        self.l_d = 0.2     #Lookahead distance


        self.dt = 0.01   
        self.finish_flag = False     

        rospy.init_node('Pure_pursuit')

        rospy.Subscriber('/odom', Pose2D, self.odom_callback)
        rospy.Subscriber('/activator',String, self.activator_callback)
        rospy.Subscriber('/pure_pursuit/trajectory_selector',Int16, self.selector_callback)

        
        self.pp_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.talkback_pub = rospy.Publisher('/pure_pursuit/talkback', String, queue_size=10)
        

        self.t1 = rospy.Timer(rospy.Duration(self.dt), self.timer_callback)

        self.rate = rospy.Rate(1/self.dt)
        
        rospy.on_shutdown(self.stop)
    
    def timer_callback(self, time):
        msg = Twist()
        
        if self.activate_flag:
            rospy.loginfo("PP_activo")
            self.find_closest_wp()
            self.find_goal()
            self.transform_goal_coordenates()
            self.set_parameters()
            msg.linear.x = self.v
            msg.angular.z = self.omega

            msg.linear.y = 0.0
            msg.linear.z = 0.0
            msg.angular.x = 0.0
            msg.angular.y = 0.0
            self.pp_pub.publish(msg)

            if ((self.X - self.wp[0,-1])**2 + (self.Y - self.wp[1,-1])**2)**0.5 < 0.3:
                # if not self.finish_flag:
                rospy.loginfo("termina trayectoria")
                # self.finish_flag = True
                self.talkback_pub.publish("done")
                self.activate_flag = False
                # self.wp = np.array([np.linspace(0.8,0.8,15),np.linspace(0.35,0.1,15)])
                # self.wp = np.append(self.wp, np.array([np.linspace(0.8,-0.3,15),np.linspace(0.1,0.1,15)]),axis=1)
        
    def selector_callback(self,msg):
        if msg.data == 1:
            self.wp = np.array([np.linspace(0.5,1.40,20),np.linspace(0,0,20)])
        elif msg.data == 2:
            self.wp = np.array([np.linspace(0.7,0.7,15),np.linspace(0.35,0.1,15)])
            self.wp = np.append(self.wp, np.array([np.linspace(0.7,0.2,15),np.linspace(0.09,0.07,15)]),axis=1)
            

    def activator_callback(self,msg):
        if msg.data == "PP_activate":
            self.activate_flag = True

    def odom_callback(self,msg): #Determine the current location and direction
        self.X = msg.x
        self.Y = msg.y
        self.theta = msg.theta
        np.append(self.pos, [[msg.x], [msg.y]], axis=1)

    def find_closest_wp(self): #Find the index of the waypoint closest to the vehicle
        distances = self.wp - np.array([[self.X], [self.Y]])
        self.hypothenuses = np.hypot(distances[0,:], distances[1,:])
        self.actual_wp = np.argmin(self.hypothenuses)
        
    def find_goal(self): #Find the goal point
        next_index = self.actual_wp+1 if self.actual_wp < self.wp.shape[1]-1 else self.actual_wp
        relevant_distances = self.hypothenuses[next_index:self.wp.shape[1]] - self.l_d
        self.next_wp = np.argmin(np.abs(relevant_distances)) + next_index

    def transform_goal_coordenates(self):
        R_t = np.transpose(np.array([[np.cos(self.theta), -np.sin(self.theta)], [np.sin(self.theta), np.cos(self.theta)]])) #Rotacion para ir del sistema global al del carro
        d = np.array([[self.X],[self.Y]])
        self.goal_transformed = R_t.dot(self.wp[:,self.next_wp].reshape(2,1)) - R_t.dot(d)

    def set_parameters(self):
        self.omega = self.v * 2*float(self.goal_transformed[1]) / self.l_d**2
        if self.omega > self.max_omega:
            self.omega = self.max_omega

    def run(self):
        rospy.spin()
            

    def stop(self):
        rospy.loginfo("Stopping pure pursuit")
        # self.talkback_pub.publish("done")
        # self.t1.shutdown()
        t = Twist()
        t.linear.x = 0
        t.angular.z = 0
        self.pp_pub.publish(t)

if __name__ == '__main__':
    pp = Pure_pursuit()

    try:
        pp.run()
    except:
        pass

