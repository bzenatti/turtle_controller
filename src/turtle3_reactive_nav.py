#!/usr/bin/env python3

import rclpy
import math
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan  
from tf_transformations import euler_from_quaternion

class Controller_Node(Node):
    def __init__(self):
        super().__init__('turtle_controller')
        self.get_logger().info("Node Started")
        
        self.max_vel = 0.1   
        self.max_angular_speed = 0.5
        self.desired_x = -0.6  # x goal
        self.desired_y = 0.4  # y goal
        self.avoidance_active = False  
        self.avoidance_threshold = 0.5  # Threshold distance to trigger avoidance (meters)

        self.front_distance = 0
        self.left_distance = 0
        self.right_distance = 0
        
        # Publisher and Subscribers
        self.odom_sub = self.create_subscription(Odometry, "/odom", self.pose_callback, 10)
        self.laser_sub = self.create_subscription(LaserScan, "/scan", self.laser_callback, 10)  
        self.vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)


    def laser_callback(self, msg):
        """Handles laser scan data and determines if obstacle avoidance is needed."""
        if not msg.ranges:
            return

        # Find the closest distances, left and right are actually diagonals, range of 30-60 degrees
        min_front_distance = min([d for i, d in enumerate(msg.ranges) if msg.range_min < d < msg.range_max and math.radians(-30) <= msg.angle_min + i * msg.angle_increment <= math.radians(30)], default=float('inf'))
        min_left_distance = min([d for i, d in enumerate(msg.ranges) if msg.range_min < d < msg.range_max and math.radians(30) < msg.angle_min + i * msg.angle_increment <= math.radians(60)], default=float('inf'))
        min_right_distance = min([d for i, d in enumerate(msg.ranges) if msg.range_min < d < msg.range_max and math.radians(-60) <= msg.angle_min + i * msg.angle_increment < math.radians(-30)], default=float('inf'))

        if min_front_distance < self.avoidance_threshold:
            self.avoidance_active = True

            # Turn in the opposite direction of the obstacle
            if min_right_distance < min_left_distance:
                avoidance_angular_velocity = 1 / min_right_distance  # Turn left
                self.get_logger().info(f"Turning left (obstacle on right at {min_right_distance})")
            else:
                avoidance_angular_velocity = -1 / min_left_distance  # Turn right
                self.get_logger().info(f"Turning right (obstacle on left at {min_left_distance})")

            # Limit angular velocity
            avoidance_angular_velocity = max(min(avoidance_angular_velocity, self.max_angular_speed), -self.max_angular_speed)

            self.my_velocity_cont(0.0, avoidance_angular_velocity)

        elif min_left_distance < self.avoidance_threshold or min_right_distance < self.avoidance_threshold:
            # If an obstacle is on either side, don't resume proportional control
            self.avoidance_active = True

            if min_right_distance < min_left_distance:
                avoidance_angular_velocity = 1 / min_right_distance  # Turn left
                self.get_logger().info(f"Turning left (obstacle on right at {min_right_distance})")
            else:
                avoidance_angular_velocity = -1 / min_left_distance  # Turn right
                self.get_logger().info(f"Turning right (obstacle on left at {min_left_distance})")

            # Limit angular velocity
            avoidance_angular_velocity = max(min(avoidance_angular_velocity, self.max_angular_speed), -self.max_angular_speed)

            # The difference here is that it goes forward on the "corridor"
            self.my_velocity_cont(0.1, avoidance_angular_velocity)

            self.get_logger().info(f"Obstacle detected on side (Left: {min_left_distance} m, Right: {min_right_distance} m)")

        else:
            self.get_logger().info("Path clear, resuming to target position")
            self.avoidance_active = False


    def pose_callback(self, msg):
        
        if self.avoidance_active:
            self.get_logger().info("Avoiding obstacle")
            return 

        self.last_pose_x = msg.pose.pose.position.x
        self.last_pose_y = msg.pose.pose.position.y

        quaternion = msg.pose.pose.orientation
        quaternion_list = [quaternion.x, quaternion.y, quaternion.z, quaternion.w]

        _, _, self.last_pose_theta = euler_from_quaternion(quaternion_list)

        # Calculate errors in position
        err_x = self.desired_x - self.last_pose_x
        err_y = self.desired_y - self.last_pose_y
        err_dist = (err_x**2 + err_y**2)**0.5  
        
        self.get_logger().info(f"Error in x {err_x} and error in y {err_y}")

        # Desired heading based on the position error
        desired_theta = math.atan2(err_y, err_x)
        
        # Error in heading
        err_theta = desired_theta - self.last_pose_theta
        err_theta = (err_theta + math.pi) % (2 * math.pi) - math.pi  # Normalize to [-pi, pi]
        self.get_logger().info(f"Desired Angle = {desired_theta} current angle {self.last_pose_theta} Error angle {err_theta}")

        # Proportional control constants
        Kp_dist = 0.4
        Kp_theta = 2
        
        # Proportional control for linear and angular velocity
        l_v = Kp_dist * abs(err_dist) 
        a_v = Kp_theta * err_theta  

        if l_v > self.max_vel:
            l_v = self.max_vel

        # Send the velocities
        self.my_velocity_cont(l_v, a_v / 5)

    def my_velocity_cont(self, l_v, a_v):
        """Publishes velocity commands to the robot."""
        self.get_logger().info(f"Commanding linear={l_v} and angular={a_v}")
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