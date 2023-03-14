#!/usr/bin/env python

import rclpy
from math import pi, sqrt, atan2
import numpy as np
from rclpy.node import Node
from std_msgs.msg import Float64
from sensor_msgs.msg import LaserScan
from lab7785_msgs.msg import BotState
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, Twist
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy, qos_profile_sensor_data

KP_LIN = 0.5
KP_ANG = 0.5

class BotBehaviour(Node):

    def __init__(self):

        print("Behaviour Node Started....")

        super().__init__("bot_behavior")

        self.waypts = []

        self.current_waypt = None

        self.tol_list = [0.02, 0.15, 0.2]

        # Read Waypoints from the file
        print("Reading Waypoints")
        with open('/home/himanshu/hr_7785/src/hr13_navigate_to_goal/hr13_navigate_to_goal/wayPoints.txt') as file:
            content = file.readlines()
            print("Content: ", content)
            self.storeWaypts(content)

        # #Odom subscriber
        self.odom_subs = self.create_subscription(Odometry, 
                                                  "/odom", 
                                                  self.callback_odom,
                                                  1)
        
        self.posx = None
        self.posy = None
        self.theta = None
        self.Init = True
        
        self.Init_posx = 0.0
        self.Init_posy = 0.0
        self.Init_posz = 0.0

        self.odom_updated = False

        #Robot State Subsriber
        self.state_subs = self.create_subscription(BotState,
                                                   "/bot_state",
                                                   self.callback_bot_state,
                                                   1)
        
        #Twist Publisher
        self.twist_pub = self.create_publisher(Twist,
                                               "/cmd_vel",
                                               10)


    # def callback_odom(self, msg):
    #     # print("Odom Callback...")
    #     self.posx = round(msg.pose.pose.position.x,4)
    #     self.posy = round(msg.pose.pose.position.y,4)
    #     self.theta = round(msg.pose.pose.orientation.z,4)
    #     print("X Pos: ", self.posx)
    #     print("Y Pos: ", self.posy)
    #     print("Theta Pos: ", self.theta)
    #     self.odom_updated = True

    def callback_bot_state(self, msg):

        if self.odom_updated:

            if msg.state == BotState.GTG:
                # print("Go To Goal Behaviour...")
                self.move2WayPt()
            elif msg.state == BotState.AO:
                print("Avoid Obstacles.....")
            elif msg.state == BotState.FW:
                print("Follow Wall.....")

    def euclidean_distance(self, des_pt):
        """
        Euclidean distance between current pose and the goal.
        """
        # return sqrt(pow((des_pt[0] - self.posx), 2) +
        #                pow((des_pt[1] - self.posy), 2))
    
        return sqrt(pow((des_pt[0] - self.posx), 2) )

    def callback_odom(self,Odom):
        print("Odom callback")
        position = Odom.pose.pose.position
        
        #Orientation uses the quaternion aprametrization.
        #To get the angular position along the z-axis, the following equation is required.
        q = Odom.pose.pose.orientation
        orientation = np.arctan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))

        if self.Init:
            #The initial data is stored to by subtracted to all the other values as we want to start at position (0,0) and orientation 0
            self.Init = False
            self.Init_ang = orientation
            self.globalAng = self.Init_ang
            Mrot = np.matrix([[np.cos(self.Init_ang), np.sin(self.Init_ang)],[-np.sin(self.Init_ang), np.cos(self.Init_ang)]])        
            self.Init_posx = Mrot.item((0,0))*position.x + Mrot.item((0,1))*position.y
            self.Init_posy = Mrot.item((1,0))*position.x + Mrot.item((1,1))*position.y
            self.Init_posz = position.z

        Mrot = np.matrix([[np.cos(self.Init_ang), np.sin(self.Init_ang)],[-np.sin(self.Init_ang), np.cos(self.Init_ang)]])        

        # #We subtract the initial values
        # self.globalPos.x = Mrot.item((0,0))*position.x + Mrot.item((0,1))*position.y - self.Init_pos.x
        # self.globalPos.y = Mrot.item((1,0))*position.x + Mrot.item((1,1))*position.y - self.Init_pos.y
        # self.globalAng = orientation - self.Init_ang

        self.posx = Mrot.item((0,0))*position.x + Mrot.item((0,1))*position.y - self.Init_posx
        self.posy = Mrot.item((1,0))*position.x + Mrot.item((1,1))*position.y - self.Init_posy
        self.theta = orientation - self.Init_ang
        self.odom_updated = True
    
    def steering_angle(self, des_pt):
           return atan2(des_pt[1] - self.posy, des_pt[0] - self.posx)

    def move2WayPt(self):
        
        des_pt = self.waypts[self.current_waypt - 1]
        #des_pt[0] += 0.1
        curr_tol = self.tol_list[self.current_waypt - 1]
        print(f"Moving to goal {self.current_waypt} with tolerance {curr_tol}")
    
        msg = Twist()
        print(f"Self Euclidiean Distance : {self.euclidean_distance(des_pt)}")
        if self.euclidean_distance(des_pt) >= curr_tol:
           
            # print("enter while")
            # Linear velocity in the x-axis.
            linear_x = KP_LIN * self.euclidean_distance(des_pt)
            if linear_x > 0.2:
                msg.linear.x = 0.2
            else:
                msg.linear.x = linear_x
            msg.linear.y = 0.0
            msg.linear.z = 0.0
 
            # Angular velocity in the z-axis.
            msg.angular.x = 0.0
            msg.angular.y = 0.0
            msg.angular.z = KP_ANG * (self.steering_angle(des_pt) - self.theta)

            self.twist_pub.publish(msg)
        else:
            # Stopping our robot after the movement is over.
            print("Reached goal")
            msg.linear.x = 0.0
            msg.angular.z = 0.0
            # self.current_waypt += 1
            self.twist_pub.publish(msg)
        


    def storeWaypts(self, content):
        for c in content:
            pts = c.split(" ")
            pts = [float(pt) for pt in pts]
            self.waypts.append(pts)

        self.current_waypt = 1
        print("Waypoints: ", self.waypts)
        print("Reading Waypoints Done.")


def main():
    rclpy.init() #init routine needed for ROS2.
    behaviour_node = BotBehaviour()

    rclpy.spin(behaviour_node)

    #Clean up and shutdown.
    behaviour_node.destroy_node()  
    rclpy.shutdown()


if __name__ == '__main__':
    main()


    