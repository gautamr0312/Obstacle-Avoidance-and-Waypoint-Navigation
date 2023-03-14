#!/usr/bin/env python

import rclpy 
from rclpy.node import Node
from std_msgs.msg import String


class DebugNode(Node):

    def __init__(self):

        super().__init__("debug_node")

        self.robot_mode_subs = self.create_subscription(String, "/robot_mode", self.callback_robot_mode_subs, 1)

        self.go_to_goal_state_subs = self.create_subscription(String, "/go_to_goal_state", self.callback_go_to_goal_subs, 1)

        self.wall_follow_state_subs = self.create_subscription(String, "/wall_follow_state", self.callback_wall_follow_subs, 1)


    def callback_robot_mode_subs(self, msg):
        print("--------------------")
        print(f'robot_mode: {msg.data}')
    
    def callback_go_to_goal_subs(self, msg):
        print("--------------------")
        print(f'go_to_goal_state : {msg.data}')

    def callback_wall_follow_subs(self, msg):
        print("--------------------")
        print(f'wall_follow_state : {msg.data}')



def main():
    rclpy.init() #init routine needed for ROS2.
    debug_node = DebugNode()

    rclpy.spin(debug_node)

    #Clean up and shutdown.
    debug_node.destroy_node()  
    rclpy.shutdown()


if __name__ == '__main__':
    main()

        

