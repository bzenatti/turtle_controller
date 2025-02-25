#!/usr/bin/env python3

#Adapted code from https://github.com/roboticvedant/ROS2_turtlesim_PID_demo/blob/main/src/turtle_demo_controller/turtle_demo_controller/turtle_controller.py
#to turtlebot3 Proportinal controller

import rclpy
import math
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf_transformations import euler_from_quaternion

class Controller_Node(Node):
    def __init__(self):
        super().__init__('turtle_controller')
        self.get_logger().info("Node Started")
        
        self.max_vel = 0.1   #Set max velocity
        self.desired_x = -0.6  # Adjust as needed
        self.desired_y = 0.4  # Adjust as needed

        # Publisher and Subscriber
        self.odom_sub = self.create_subscription(Odometry, "/odom", self.pose_callback, 10)
        self.vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)

    def pose_callback(self, msg: Odometry):

        self.last_pose_x = msg.pose.pose.position.x
        self.last_pose_y = msg.pose.pose.position.y

        quaternion = msg.pose.pose.orientation

        quaternion_list = [quaternion.x, quaternion.y, quaternion.z, quaternion.w]

        _, _, self.last_pose_theta = euler_from_quaternion(quaternion_list)

        # self.get_logger().info(f"Current x={msg.x} current y={msg.y} and current angle = {msg.theta}")
        # Calculate errors in position
        err_x = self.desired_x - self.last_pose_x
        err_y = self.desired_y - self.last_pose_y
        err_dist = (err_x**2+err_y**2)**0.5
        
        # Distance error (magnitude of the error vector)
        
        self.get_logger().info(f"Error in x {err_x} and error in y {err_y}")

        # Desired heading based on the position error
        desired_theta = math.atan2(err_y, err_x)
        
        # Error in heading
        err_theta = desired_theta - self.last_pose_theta
       
        # Handle wrap-around issues (e.g., if error jumps from +pi to -pi)
        while err_theta > math.pi:
            err_theta -= 2.0 * math.pi
        while err_theta < -math.pi:
            err_theta += 2.0 * math.pi

        self.get_logger().info(f"Desired Angle = {desired_theta} current angle {self.last_pose_theta} Error angle {err_theta}")

        Kp_dist = 0.4
            
        Kp_theta = 2
        
        # Proportional control for linear velocity
        l_v = Kp_dist * abs(err_dist) 


        # Proportional control for angular velocity
        a_v = Kp_theta * err_theta  

        if l_v > self.max_vel:
            l_v = self.max_vel

        # Send the velocities
        self.my_velocity_cont(l_v, a_v)

    def my_velocity_cont(self, l_v, a_v):
        self.get_logger().info(f"Commanding liner ={l_v} and angular ={a_v}")
        my_msg = Twist()
        my_msg.linear.x = l_v
        my_msg.angular.z = a_v
        self.vel_pub.publish(my_msg)

def main(args=None):
    rclpy.init(args=args)
    node = Controller_Node()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
