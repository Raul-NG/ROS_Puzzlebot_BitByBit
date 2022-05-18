#!/usr/bin/env python


import rospy
import numpy as np
from geometry_msgs.msg import Pose2D
from geometry_msgs.msg import Twist

class Pure_pursuit:

    def __init__(self):
        self.X = 0.0
        self.Y = 0.0
        self.theta = 0.0
        self.odom = Pose2D()
        self.actual_wp = 0 #Index for the closest waypoint
        self.next_wp = 0
        self.goal_transformed = np.array([[],[]])
        

        #RRT2
        self.wp = np.array([[0.025,0.025],
                           [0.075,0.075],
                           [0.125,0.125],
                           [0.175,0.175],
                           [0.225,0.225],
                           [0.275,0.275],
                           [0.325,0.325],
                           [0.375,0.375],
                           [0.425,0.425],
                           [0.475,0.425],
                           [0.525,0.475],
                           [0.575,0.525],
                           [0.625,0.575],
                           [0.675,0.625],
                           [0.725,0.675],
                           [0.775,0.725],
                           [0.825,0.775],
                           [0.875,0.825],
                           [0.925,0.875],
                           [0.975,0.925],
                           [1.025,0.975],
                           [1.075,1.025],
                           [1.125,1.075],
                           [1.175,1.125],
                           [1.225,1.175],
                           [1.275,1.225],
                           [1.325,1.225],
                           [1.375,1.225],
                           [1.425,1.225],
                           [1.475,1.225],
                           [1.525,1.225],
                           [1.575,1.225],
                           [1.625,1.225],
                           [1.675,1.225],
                           [1.725,1.225],
                           [1.775,1.225],
                           [1.825,1.225],
                           [1.875,1.225],
                           [1.925,1.225],
                           [1.975,1.225],
                           [2.025,1.225],
                           [2.075,1.225],
                           [2.125,1.225],
                           [2.175,1.225],
                           [2.225,1.225],
                           [2.275,1.225],
                           [2.325,1.225],
                           [2.375,1.225],
                           [2.425,1.225]])

        self.wp = np.transpose(self.wp)
        self.wp = self.wp - self.wp[:,0].reshape(2,1)
        self.wp = np.array([[np.cos(np.pi/2), -np.sin(np.pi/2)], [np.sin(np.pi/2), np.cos(np.pi/2)]]).dot(self.wp)




        #Trapecio
       # self.wp = np.array([np.linspace(0,1.8,20),np.linspace(0,0,20)])
       # self.wp = np.append(self.wp, np.array([np.linspace(1.8,1.3,20),np.linspace(0.2,1,20)]),axis=1)
       # self.wp = np.append(self.wp, np.array([np.linspace(1.3,0.3,20),np.linspace(1,1,20)]),axis=1)
       # self.wp = np.append(self.wp, np.array([np.linspace(0.3,0,20),np.linspace(1,0.4,20)]),axis=1)


        #Triangulo
       # self.wp = np.array([np.linspace(0,1.8,20),np.linspace(0,0,20)])
       # self.wp = np.append(self.wp, np.array([np.linspace(1.8,0.2,20),np.linspace(0.2,1.8,20)]),axis=1)
       # self.wp = np.append(self.wp,np.array([np.linspace(0,0,20),np.linspace(1.8,0.35,20)]),axis=1)
    
       
        #Cuadrado
        #self.wp = np.array([np.linspace(0,2,20), np.linspace(0,0,20)])
        #self.wp = np.append(self.wp,np.array([np.linspace(2,2,20),np.linspace(0,2,20)]),axis=1)
        #self.wp = np.append(self.wp,np.array([np.linspace(2,0,20),np.linspace(2,2,20)]),axis=1)
        #self.wp = np.append(self.wp,np.array([np.linspace(0,0,18),np.linspace(2,0.5,18)]),axis=1)

        #self.wp = [[0, 0.15, 0.15, 0.15, 0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0], [np.linspace(0,3,20)]]
        #self.wp = np.array([[np.linspace(0,1,20),np.linspace(1,1,20)], [np.linspace(0,0,20),np.linspace(0,1,20)]])
        #self.wp = np.array([[np.linspace(0,1,20), np.linspace(0,0,20)], [np.linspace(1,1,20), np.linspace(0,1,20)], [np.linspace(1,0,20), np.linspace(1,1,20)],[np.linspace(0,0,20), np.linspace(1,0.2,20)]])
        
        self.v = 0.4      #Linear velocity
        self.omega = 0.0      #Angular velocity
        self.pos = np.array([[0],[0]])       #Position array

        self.max_omega = 1    #Maximum angular velocity
        self.l_d = 0.3     #Lookahead distance


        self.dt = 0.01        

        rospy.init_node('Pure_pursuit')

        rospy.Subscriber('/odom', Pose2D, self.odom_callback)
        
        self.pp_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        self.t1 = rospy.Timer(rospy.Duration(self.dt), self.timer_callback)

        self.rate = rospy.Rate(100)
        
        rospy.on_shutdown(self.stop)
    
    def timer_callback(self, time):
        msg = Twist()

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
        while not rospy.is_shutdown():
            if ((self.X - self.wp[0,-1])**2 + (self.Y - self.wp[1,-1])**2)**0.5 < 0.3:
                self.stop()
            self.rate.sleep()

    def stop(self):
        rospy.loginfo("Stopping pure pursuit")
        self.t1.shutdown()
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

