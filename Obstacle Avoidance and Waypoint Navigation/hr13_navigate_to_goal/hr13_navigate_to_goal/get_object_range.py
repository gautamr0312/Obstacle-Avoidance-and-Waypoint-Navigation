#!/usr/bin/env python

import rclpy
from math import pi
import numpy as np
from rclpy.node import Node
from std_msgs.msg import Float64
from sensor_msgs.msg import LaserScan
from lab7785_msgs.msg import BotState
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy, qos_profile_sensor_data

PI = pi
TOL = 0.02
AOV = 15 * (PI / 180)

class LidarNode(Node):

    def __init__(self):

        print("Lidar Node Started....")

        super().__init__("object_range")
        
        self.declare_parameter("min_obs_dist", 0.20)
        


        self.lidar_subs = self.create_subscription(LaserScan,
                                                   "/scan",
                                                   self.callback_lidar_range,
                                                   qos_profile_sensor_data)
        
        self.state_pub = self.create_publisher(BotState,
                                               "/bot_state",
                                               10)
        
        self.state_pub_timer = self.create_timer(0.1, self.callback_state_pub)


        self.lidar_subs

        self.current_state = BotState.GTG

    def callback_state_pub(self):
        msg = BotState()
        msg.state = self.current_state
        self.state_pub.publish(msg)

    def callback_lidar_range(self, msg):
        print("Callback Lidar...")
        angle_min = msg.angle_min
        angle_max = msg.angle_max
        angle_inc = msg.angle_increment

        index = round( (abs(AOV) - angle_min ) / angle_inc)


        ranges = np.asarray(msg.ranges[:index] + msg.ranges[-index:])
        ranges = ranges[~np.isnan(ranges)]
        # print(f"Ranges : {ranges} and length : {len(ranges)}")
        min_rng_val = np.min(ranges)
        print(f"The obstacle is at {min_rng_val}")
        if min_rng_val < self.get_parameter("min_obs_dist").value + TOL:
            print("Object Close, trigger obstacle avoidance")
            self.current_state = BotState.AO


def main():
    rclpy.init() #init routine needed for ROS2.
    lidar_node = LidarNode()

    rclpy.spin(lidar_node)

    #Clean up and shutdown.
    lidar_node.destroy_node()  
    rclpy.shutdown()


if __name__ == '__main__':
    main()





