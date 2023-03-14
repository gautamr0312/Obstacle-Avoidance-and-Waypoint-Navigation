#!/usr/bin/env python

import time
import rclpy 
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data 
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan  
from geometry_msgs.msg import Twist  
from geometry_msgs.msg import Pose 
from nav_msgs.msg import Odometry
import numpy as np 
import math

class ControllerNode(Node):

    def __init__(self):
        super().__init__("controller_node")

        #Odom subscriber
        self.odom_subs = self.create_subscription(Odometry, 
                                                  "/odom", 
                                                  self.callback_odom,
                                                  1)
        
        #Twist Publisher
        self.twist_pub = self.create_publisher(Twist,
                                               "/cmd_vel",
                                               10)
        # Lidar Subsriber
        self.lidar_subs = self.create_subscription(LaserScan,
                                                   "/scan",
                                                   self.callback_scan,
                                                   qos_profile_sensor_data)
        

        self.robot_mode_pub = self.create_publisher(String, 
                                                    "/robot_mode",
                                                    10)
        
        self.go_to_goal_state_pub = self.create_publisher(String, 
                                                          "/go_to_goal_state",
                                                          10)
        
        self.wall_following_state_pub = self.create_publisher(String,
                                                              "/wall_follow_state",
                                                              10)
        
        self.timer_pub = self.create_timer(0.2, self.callback_timer)
        
        # Flag use to reset the position values
        self.Init = True

        # Initial Values of Pose
        self.Init_posx = 0.0
        self.Init_posy = 0.0
        self.Init_posz = 0.0

        # Current position and orientation of the robot in the global 
        # reference frame
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_theta = 0.0

        # We get the goal coordinates from Waypoints.txt
        self.goal_x = [ 1.7, 1.7, 0.2]
        self.goal_y = [ 0.0, 1.4, 1.4]

        # By changing the value of self.robot_mode, you can alter what
        # the robot will do when the program is launched.
        #   "obstacle avoidance mode": Robot will avoid obstacles
        #   "go to goal mode": Robot will head to an x,y coordinate   
        #   "wall following mode": Robot will follow a wall 
        self.robot_mode = "go to goal mode"

        # Initialize the LaserScan sensor readings to some large value
        # Values are in meters.
        self.left_dist = np.inf # Left
        self.leftfront_dist = np.inf # Left-front
        self.front_dist = np.inf # Front
        self.rightfront_dist = np.inf # Right-front
        self.right_dist = np.inf # Right
         
        # Distance between the leave point and the goal in meters
        self.distance_to_goal_from_leave_point = 0.0

        # Keep track of which goal we're headed towards
        self.goal_idx = 0
         
        # Keep track of when we've reached the end of the goal list
        self.goal_max_idx =  2 # len(self.goal_x_coordinates) - 1 

        # Finite states for the go to goal mode
        #   "adjust heading": Orient towards a goal x, y coordinate
        #   "go straight": Go straight towards goal x, y coordinate
        #   "goal achieved": Reached goal x, y coordinate
        self.go_to_goal_state = "adjust heading"

        # +/- 2.0 degrees of precision
        self.theta_precision = 5.0 * (math.pi / 180) 

        # How quickly we need to turn when we need to make a heading
        # adjustment (rad/s)
        self.turning_speed_theta_adjustment = 0.50

        # Need to get within +/- 0.2 meter (20 cm) of (x,y) goal
        self.dist_precision = 0.1

        # Start-Goal Line Calculated?
        self.start_goal_line_calculated = False
         
        # Start-Goal Line Parameters
        self.start_goal_line_slope_m = 0
        self.start_goal_line_y_intercept = 0
        self.start_goal_line_xstart = 0
        self.start_goal_line_xgoal = 0
        self.start_goal_line_ystart = 0
        self.start_goal_line_ygoal = 0

        # Anything less than this distance means we have encountered
        # a wall. Value determined through trial and error.
        self.dist_thresh_bug2 = 0.30
         

        # Used to record the (x,y) coordinate where the robot hit
        # a wall.
        self.hit_point_x = 0
        self.hit_point_y = 0
         
        # Distance between the hit point and the goal in meters
        self.distance_to_goal_from_hit_point = 0.0
         
        # Used to record the (x,y) coordinate where the robot left
        # a wall.       
        self.leave_point_x = 0
        self.leave_point_y = 0

        # Finite states for the wall following mode
        #   "turn left": Robot turns towards the left
        #   "search for wall": Robot tries to locate the wall       
        #   "follow wall": Robot moves parallel to the wall
        self.wall_following_state = "turn left"
        

        #Laser Tolerance
        self.laser_angles = np.array([0.0, 45.0, 90.0, 270.0, 325.0]) * (np.pi / 180 )
        self.laser_tolerance = 5

        # Controller 
        self.kp_lin = 0.5
        self.kp_ang = 0.25

        # All Threshold Speeds

        # Maximum forward speed of the robot in meters per second
        # Any faster than this and the robot risks falling over.
        self.forward_speed = 0.2

        self.forward_speed_fw = 0.1

        # Maximum left-turning speed    
        self.turning_speed = 1.0 # rad/s

        # Set turning speeds (to the left) in rad/s 
        # These values were determined by trial and error.
        self.turning_speed_wf_fast = 1.00  # Fast turn
        self.turning_speed_wf_slow = 0.25  # Slow turn

        # All Precision Values

        # Obstacle detection distance threshold
        self.dist_thresh_obs = 0.15 # in meters

        # The hit point and leave point must be far enough 
        # apart to change state from wall following to go to goal
        # This value helps prevent the robot from getting stuck and
        # rotating in endless circles.
        # This distance was determined through trial and error.
        self.leave_point_to_hit_point_diff = 0.25 # in meters

        # Leave point must be within +/- 0.1m of the start-goal line
        # in order to go from wall following mode to go to goal mode
        self.distance_to_start_goal_line_precision = 0.20

        # Wall following distance threshold.
        # We want to try to keep within this distance from the wall.
        self.dist_thresh_wf = 0.50 # in meters  

        # We don't want to get too close to the wall though.
        self.dist_too_close_to_wall = 0.40 # in meters


    def callback_timer(self):
        msg = String()
        msg.data = self.robot_mode
        self.robot_mode_pub.publish(msg)   

        msg = String()
        msg.data = self.go_to_goal_state
        self.go_to_goal_state_pub.publish(msg) 

        msg = String()
        msg.data = self.wall_following_state
        self.wall_following_state_pub.publish(msg)     

    def callback_odom(self, Odom):
        # print("Odom callback")
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

        self.current_x = Mrot.item((0,0))*position.x + Mrot.item((0,1))*position.y - self.Init_posx
        self.current_y = Mrot.item((1,0))*position.x + Mrot.item((1,1))*position.y - self.Init_posy
        self.current_theta = orientation - self.Init_ang

        if self.goal_x == False and self.goal_y == False:
            return
        
        #Call Bug2 Algorithm
        self.bug2_algo()

        if self.robot_mode == "go to goal mode":
            self.go_to_goal()
        elif self.robot_mode == "wall following mode":
            self.follow_wall()
        else:
            pass # Do nothing    


    def callback_scan(self, msg):

        angle_min = msg.angle_min
        angle_inc = msg.angle_increment
        len_rng_arr = len(msg.ranges)
        distance = []
        for angle in self.laser_angles:

            index = round( (abs(angle) - angle_min ) / angle_inc) 
            index_min = round(index - self.laser_tolerance)
            index_max = round(index + self.laser_tolerance)

            if index_min < 0 :
                index_min = len_rng_arr + index_min
                rng_arr = np.array(msg.ranges[index_min:len_rng_arr]+msg.ranges[0:index_max])

            elif index_max > len_rng_arr:
                index_max = index_max - len_rng_arr
                rng_arr = np.array(msg.ranges[index_min:len_rng_arr]+msg.ranges[0:(index_max-len_rng_arr)])
                
            else:
                rng_arr = np.array(msg.ranges[index_min:index_max])
                

            rng_arr =rng_arr[~np.isnan(rng_arr)]
            dist = float(np.mean(rng_arr))
            distance.append(dist)

        self.front_dist = distance[0] # 0.0
        self.leftfront_dist = distance[1] # 45.0
        self.left_dist = distance[2] #90.0
        self.right_dist = distance[3] # 270.0
        self.rightfront_dist = distance[4] # 325

        # print(f"{ self.rightfront_dist, self.leftfront_dist}")
        # self.leftfront_dist , self.front_dist, self.rightfront_dist,


        # d = self.dist_thresh_obs
        # if  self.leftfront_dist < d and self.front_dist < d and self.rightfront_dist < d:

        #     # Change the mode to wall following mode.
        #     self.robot_mode = "obstacle avoidance mode"

        #     self.avoid_obstacles()

        # elif self.leftfront_dist > d and self.front_dist > d and self.rightfront_dist > d:
        #     self.robot_mode = "go to goal mode"


        # if self.robot_mode == "obstacle avoidance mode":
        #     self.avoid_obstacles()




    def avoid_obstacles(self):

        msg = Twist()
        msg.linear.x = 0.0
        msg.linear.y = 0.0
        msg.linear.z = 0.0
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = 0.0

        d = self.dist_thresh_obs

        if self.leftfront_dist > d and self.front_dist > d and self.rightfront_dist > d :
            # Obstacle is not in front of robot
            msg.linear.x = self.forward_speed # Go straight forward
        elif self.leftfront_dist > d and self.front_dist < d and self.rightfront_dist > d:
            # Obstacle in front of the robot
            msg.angular.z = self.turning_speed  # Turn left
        elif self.leftfront_dist > d and self.front_dist > d and self.rightfront_dist < d:
            # Obstacle in rightfront of the robot
            msg.angular.z = self.turning_speed  # Turn left
        elif self.leftfront_dist < d and self.front_dist > d and self.rightfront_dist > d:
            # Obstacle in leftfront of the robot
            msg.angular.z = -self.turning_speed # Turn right
        elif self.leftfront_dist > d and self.front_dist < d and self.rightfront_dist < d:
            # Obstacle in Forward & Rightfront
            msg.angular.z = self.turning_speed  # Turn Left
        elif self.leftfront_dist < d and self.front_dist < d and self.rightfront_dist > d:
            # Obstacle in Forward & Leftfront
            msg.angular.z = -self.turning_speed
        elif self.leftfront_dist < d and self.front_dist < d and self.rightfront_dist < d:
            # Obstacle in Front RightFront LeftFront
            msg.angular.z = self.turning_speed # Turn Left
        elif self.leftfront_dist < d and self.front_dist > d and self.rightfront_dist < d:
            # Obstacle in Leftfront & RightFront
            msg.linear.x = self.forward_speed
        else:
            pass

        self.twist_pub.publish(msg)

    
    def go_to_goal(self):

        msg = Twist()
        msg.linear.x = 0.0
        msg.linear.y = 0.0
        msg.linear.z = 0.0
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = 0.0

        # If the wall is in the way
        d = self.dist_thresh_bug2
        
        if  self.leftfront_dist < d or self.front_dist < d or self.rightfront_dist < d:
            # Change the mode to wall following mode.
            self.robot_mode = "wall following mode"

            # Record the hit point  
            self.hit_point_x = self.current_x
            self.hit_point_y = self.current_y

            # Record the distance to the goal from the 
            # hit point
            self.distance_to_goal_from_hit_point = (
                math.sqrt((
                pow(self.goal_x[self.goal_idx] - self.hit_point_x, 2)) + (pow(self.goal_y[self.goal_idx] - self.hit_point_y, 2))))

            # Make a hard left to begin following wall
            msg.angular.z = self.turning_speed_wf_fast  

            self.twist_pub.publish(msg)

            return
        
        # Fix the heading  
        if self.go_to_goal_state == "adjust heading":
            # Calculate the desired heading based on the current position 
            # and the desired position
            desired_theta = math.atan2(
                    self.goal_y[self.goal_idx] - self.current_y,
                    self.goal_x[self.goal_idx] - self.current_x)
            
            
            # How far off is the current heading in radians?        
            theta_error = desired_theta - self.current_theta
            # if theta_error > np.pi :
            #     theta_error = 

            if theta_error <= -np.pi:
                theta_error += 2*np.pi
            elif theta_error > np.pi:
                theta_error -= 2*np.pi
            print("Theta Error:", theta_error )

             # Adjust heading if heading is not good enough
            if math.fabs(theta_error) > self.theta_precision:

                if theta_error > 0:    
                    # Turn left (counterclockwise)      
                    msg.angular.z = self.turning_speed_theta_adjustment       
                else:
                    # Turn right (clockwise)
                    msg.angular.z = -self.turning_speed_theta_adjustment    

                self.twist_pub.publish(msg)

            else:
                # Change the state
                self.go_to_goal_state = "go straight"

                # Command the robot to stop turning
                self.twist_pub.publish(msg)  

        # Go straight                                       
        elif self.go_to_goal_state == "go straight":

            position_error = math.sqrt(
                        pow(
                        self.goal_x[self.goal_idx] - self.current_x, 2)
                        + pow(
                        self.goal_y[self.goal_idx] - self.current_y, 2))
            
            # If we are still too far away from the goal                        
            if position_error > self.dist_precision:

                # Move straight ahead
                # msg.linear.x = self.forward_speed
                # linear_x = self.kp_lin * position_error
                # if linear_x > 0.2:
                #     linear_x = 0.2
                
                msg.linear.x = self.forward_speed

                self.twist_pub.publish(msg)

                

                # Check our heading         
                desired_theta = math.atan2(
                    self.goal_y[self.goal_idx] - self.current_y,
                    self.goal_x[self.goal_idx] - self.current_x)
                
                # How far off is the heading?   
                theta_error = desired_theta - self.current_theta  

                # Check the heading and change the state if there is too much heading error
                if math.fabs(theta_error) > self.theta_precision:
                     
                    # Change the state
                    self.go_to_goal_state = "adjust heading"

            # We reached our goal. Change the state.
            else:           
                # Change the state
                self.go_to_goal_state = "goal achieved"
                
                # Command the robot to stop
                self.twist_pub.publish(msg)

        # Goal achieved         
        elif self.go_to_goal_state == "goal achieved":
                 
            print('Goal achieved! X:%f Y:%f' % (
                self.goal_x[self.goal_idx],
                self.goal_y[self.goal_idx]))
            
            time.sleep(10)
             
            # Get the next goal
            self.goal_idx = self.goal_idx + 1
         
            # Do we have any more goals left?           
            # If we have no more goals left, just stop
            if self.goal_idx > self.goal_max_idx:
                print('Congratulations! All goals have been achieved.')
                while True:
                    pass
 
            # Let's achieve our next goal
            else: 
                # Change the state
                self.go_to_goal_state = "adjust heading"               
                
            # We need to recalculate the start-goal line if Bug2 is running
            self.start_goal_line_calculated = False  

        else:
            pass

    
    def follow_wall(self):
        # print("Enter Follow wall")
        msg = Twist()
        msg.linear.x = 0.0
        msg.linear.y = 0.0
        msg.linear.z = 0.0
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = 0.0
        #print(self.leftfront_dist,self.front_dist, self.rightfront_dist)
        # Calculate the point on the start-goal 
        # line that is closest to the current position
        x_start_goal_line = self.current_x
        y_start_goal_line = (
            self.start_goal_line_slope_m * (
            x_start_goal_line)) + (
            self.start_goal_line_y_intercept)
        
        # Calculate the distance between current position 
        # and the start-goal line
        distance_to_start_goal_line = math.sqrt(pow(
                    x_start_goal_line - self.current_x, 2) + pow(
                    y_start_goal_line - self.current_y, 2)) 
        # print(distance_to_start_goal_line)
        # If we hit the start-goal line again               
        if distance_to_start_goal_line < self.distance_to_start_goal_line_precision:
            print("Entereddddddddddd!!!!")
            # Determine if we need to leave the wall and change the mode
            # to 'go to goal'
            # Let this point be the leave point
            self.leave_point_x = self.current_x
            self.leave_point_y = self.current_y

            # Record the distance to the goal from the leave point
            self.distance_to_goal_from_leave_point = math.sqrt(
                pow(self.goal_x[self.goal_idx] 
                - self.leave_point_x, 2)
                + pow(self.goal_y[self.goal_idx]  
                - self.leave_point_y, 2)) 
            
            # Is the leave point closer to the goal than the hit point?
            # If yes, go to goal. 
            diff = self.distance_to_goal_from_hit_point - self.distance_to_goal_from_leave_point
            if diff > self.leave_point_to_hit_point_diff:
                print("Yessssss!!")
                # Change the mode. Go to goal.
                self.robot_mode = "go to goal mode"

                return
        
        d = self.dist_thresh_wf
         
        if self.leftfront_dist > d and self.front_dist > d and self.rightfront_dist > d:
            self.wall_following_state = "search for wall"
            # self.hybrid_move_rot(self.forward_speed_fw, -self.turning_speed_wf_slow)
            msg.linear.x = self.forward_speed_fw
            msg.angular.z = -self.turning_speed_wf_slow # turn right to find wall
            
        elif self.leftfront_dist > d and self.front_dist < d and self.rightfront_dist > d:
            self.wall_following_state = "turn left"
            msg.angular.z = self.turning_speed_wf_fast
            
            
        elif (self.leftfront_dist > d and self.front_dist > d and self.rightfront_dist < d):
            if (self.rightfront_dist < self.dist_too_close_to_wall):
                # Getting too close to the wall
                self.wall_following_state = "turn left"
                # self.hybrid_move_rot(self.forward_speed_fw, self.turning_speed_wf_fast)
                # msg.linear.x = self.forward_speed_fw
                msg.angular.z = self.turning_speed_wf_fast      
            else:           
                # Go straight ahead
                self.wall_following_state = "follow wall" 
                msg.linear.x = self.forward_speed_fw   
                                    
        elif self.leftfront_dist < d and self.front_dist > d and self.rightfront_dist > d:
            self.wall_following_state = "search for wall"
            # self.hybrid_move_rot(self.forward_speed_fw, -self.turning_speed_wf_slow)

            # msg.linear.x = self.forward_speed_fw
            msg.angular.z = -self.turning_speed_wf_slow # turn right to find wall
            
        elif self.leftfront_dist > d and self.front_dist < d and self.rightfront_dist < d:
            self.wall_following_state = "turn left"
            msg.angular.z = self.turning_speed_wf_fast
            
        elif self.leftfront_dist < d and self.front_dist < d and self.rightfront_dist > d:
            self.wall_following_state = "turn left"
            msg.angular.z = self.turning_speed_wf_fast
            
        elif self.leftfront_dist < d and self.front_dist < d and self.rightfront_dist < d:
            self.wall_following_state = "turn left"
            msg.angular.z = self.turning_speed_wf_fast
            
        elif self.leftfront_dist < d and self.front_dist > d and self.rightfront_dist < d:
            self.wall_following_state = "search for wall"
            # self.hybrid_move_rot(self.forward_speed_fw, -self.turning_speed_wf_slow)
            msg.linear.x = self.forward_speed_fw
            # msg.angular.z = -self.turning_speed_wf_slow # turn right to find wall
            
        else:
            pass

        self.twist_pub.publish(msg)

    def hybrid_move_rot(self, forward_speed, turning_speed):
        
        msg = Twist()
        msg.linear.x = 0.0
        msg.linear.y = 0.0
        msg.linear.z = 0.0
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = turning_speed

        self.twist_pub.publish(msg)

        msg = Twist()
        msg.linear.x = forward_speed
        msg.linear.y = 0.0
        msg.linear.z = 0.0
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = 0.0

        self.twist_pub.publish(msg)



    def bug2_algo(self):

        # Each time we start towards a new goal, we need to calculate the start-goal line
        if self.start_goal_line_calculated == False:
         
            # Make sure go to goal mode is set.
            self.robot_mode = "go to goal mode"            
 
            self.start_goal_line_xstart = self.current_x
            self.start_goal_line_xgoal = self.goal_x[self.goal_idx]
            self.start_goal_line_ystart = self.current_y
            self.start_goal_line_ygoal = self.goal_y[self.goal_idx]
             
            # Calculate the slope of the start-goal line m
            self.start_goal_line_slope_m = (
                (self.start_goal_line_ygoal - self.start_goal_line_ystart) / (
                self.start_goal_line_xgoal - self.start_goal_line_xstart))
             
            # Solve for the intercept b
            self.start_goal_line_y_intercept = self.start_goal_line_ygoal - (
                    self.start_goal_line_slope_m * self.start_goal_line_xgoal) 
                 
            # We have successfully calculated the start-goal line
            self.start_goal_line_calculated = True
             
        if self.robot_mode == "go to goal mode":
            self.go_to_goal()           
        elif self.robot_mode == "wall following mode":
            self.follow_wall()

    # def storeWaypts(self, content):
    #     for c in content:
    #         pts = c.split(" ")
    #         pts = [float(pt) for pt in pts]
    #         self.goal_x.append(pts[0] + 0.1) # Add 10cm (Systematic Error due to wheel deformation)
    #         self.goal_y.append(pts[1])

    #     print("Goal X :", self.goal_x )
    #     print("Goal Y: ", self.goal_y)
    #     print("Reading Waypoints Done.")


        
def main(args=None):
 
    
    rclpy.init(args=args)
     
    controller = ControllerNode()
 
    rclpy.spin(controller)
 
    controller.destroy_node()
     
    rclpy.shutdown()

            
if __name__ == '__main__':
    main()





            









         
        
        
        



