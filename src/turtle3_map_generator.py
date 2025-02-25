#!/usr/bin/env python3

import rclpy
import math
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from tf_transformations import euler_from_quaternion

# Map is 5x5 meters with 50x50 cells (10 cm each). Each cell initially set to 0.
MAP_SIZE = 50
CELL_SIZE = 0.1  # 10 cm
MAP_OFFSET = MAP_SIZE // 2  # Offset to shift map indices for negative coordinates
map_grid = [[0 for _ in range(MAP_SIZE)] for _ in range(MAP_SIZE)]


class MapperNode(Node):
    def __init__(self):
        super().__init__('mapper_node')
        self.get_logger().info("Mapper Node Started")

        # Robot's initial position and orientation (turtlebot3_world)
        self.robot_x = -2.0
        self.robot_y = -0.5
        self.robot_theta = 0.0

        # Distances from LaserScan
        self.laser_ranges = []

        # Publisher and Subscribers
        self.odom_sub = self.create_subscription(Odometry, "/odom", self.pose_callback, 10)
        self.laser_sub = self.create_subscription(LaserScan, "/scan", self.laser_callback, 10)

    def laser_callback(self, msg):
        
        self.laser_ranges = msg.ranges
        angle_min = msg.angle_min
        angle_increment = msg.angle_increment

        # Iterate over all laser beams to map obstacles
        for i, distance in enumerate(self.laser_ranges):
            if msg.range_min < distance < msg.range_max:
                # Calculate beam angle relative to the robot
                beam_angle = self.robot_theta + angle_min + i * angle_increment

                # Convert polar coordinates (distance, angle) to Cartesian
                obstacle_x = self.robot_x + distance * math.cos(beam_angle)
                obstacle_y = self.robot_y + distance * math.sin(beam_angle)

                # Convert map coordinates to grid indices
                map_x, map_y = self.world_to_map(obstacle_x, obstacle_y)

                # Mark the obstacle on the map grid
                if 0 <= map_x < MAP_SIZE and 0 <= map_y < MAP_SIZE:
                    map_grid[map_x][map_y] = 1  

    def pose_callback(self, msg):
        
        # Get position
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y

        # Get orientation (quaternion to Euler)
        orientation = msg.pose.pose.orientation
        quaternion = [orientation.x, orientation.y, orientation.z, orientation.w]
        _, _, self.robot_theta = euler_from_quaternion(quaternion)

    def world_to_map(self, x, y):
        """Converts world coordinates to map grid indices."""
        map_x = int((x + 2.5) / CELL_SIZE)
        map_y = int((y + 2.5) / CELL_SIZE)
        return map_x, map_y

    def save_map(self):
        """
            Saves, in the directory that is the terminal running the node, 
        the current map to a file for debugging or visualization.
        """

        with open("map.txt", "w") as file:
            for row in map_grid:
                file.write("".join(map(str, row)) + "\n")
        self.get_logger().info("Map saved to map.txt")


def main(args=None):
    rclpy.init(args=args)
    node = MapperNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        # Save the map before shutting down
        node.save_map()
        node.get_logger().info("Shutting down mapper node.")
    node.destroy_node()
    #rclpy.shutdown()




if __name__ == '__main__':
    main()
