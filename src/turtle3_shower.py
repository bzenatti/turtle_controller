#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

# Define the room annotations.
#   top_left  = (min_x, max_y)
#   top_right = (max_x, max_y)
#   bottom_left  = (min_x, min_y)
#   bottom_right = (max_x, min_y)


def main(args=None):
    rclpy.init(args=args)

    # Create the navigator object.
    navigator = initialize_navigator()

    go_to_room("FrontDoor", navigator)

    # Navigate to the main pose of the Kitchen.
    room_name = "Kitchen"
    go_to_room(room_name, navigator)

    # 3. Wait for the navigation task to complete.
    while not navigator.isTaskComplete():
        rclpy.spin_once(navigator, timeout_sec=0.1)

    # 4. Get the result and shut down.
    result = navigator.getResult()

    if result == TaskResult.SUCCEEDED:
        navigator.get_logger().info('Navigation succeeded!')
    else:
        navigator.get_logger().error(f'Navigation failed with result: {result}')

    # Cleanup: destroy the node and shutdown rclpy.
    navigator.destroy_node()
    rclpy.shutdown()


def go_to_room(room_name, navigator):
    """Navigates the robot to the main pose of the specified room."""
    if room_name not in ROOMS:
        navigator.get_logger().error(f"Room '{room_name}' not found in predefined rooms.")
        return
    
    room = ROOMS[room_name]
    navigator.get_logger().info(f"Navigating to the {room_name}...")

    # Create a goal pose from the room's main_pose.
    goal_pose = PoseStamped()
    goal_pose.header.frame_id = 'map'
    goal_pose.header.stamp = navigator.get_clock().now().to_msg()
    goal_pose.pose.position.x = room["main_pose"]["position"]["x"]
    goal_pose.pose.position.y = room["main_pose"]["position"]["y"]
    goal_pose.pose.position.z = room["main_pose"]["position"]["z"]
    goal_pose.pose.orientation.x = room["main_pose"]["orientation"]["x"]
    goal_pose.pose.orientation.y = room["main_pose"]["orientation"]["y"]
    goal_pose.pose.orientation.z = room["main_pose"]["orientation"]["z"]
    goal_pose.pose.orientation.w = room["main_pose"]["orientation"]["w"]

    navigator.get_logger().info(f"Sending goal to {room_name} via goToPose...")
    navigator.goToPose(goal_pose)

    # Wait for the navigation task to complete.
    while not navigator.isTaskComplete():
        rclpy.spin_once(navigator, timeout_sec=0.1)

ROOMS = {
    "DiningRoom": {
        "bounding_box": {
            "top_left": {"x": -2.70, "y": 4.94},
            "top_right": {"x": 1.46,  "y": 4.94},
            "bottom_left": {"x": -2.70, "y": 0.73},
            "bottom_right": {"x": 1.46,  "y": 0.73}
        },
        "main_pose": {
            "position": {"x": 1.46, "y": 4.94, "z": 0.0},
            "orientation": {"x": 0.0, "y": 0.0, "z": -0.9535, "w": 0.30135}
        }
    },
    "FrontDoor": {
        "bounding_box": {
            "top_left": {"x": 3.0, "y": -0.8},
            "top_right": {"x": 3.2, "y": -0.8},
            "bottom_left": {"x": 3.0, "y": -1.2},
            "bottom_right": {"x": 3.2, "y": -1.2}
        },
        "main_pose": {
            "position": {"x": 3.1, "y": -1.0, "z": 0.0},
            "orientation": {"x": 0.0, "y": 0.0, "z": 0.92, "w": 1.0}
        }
    },
    "Bedroom": {
        "bounding_box": {
            "top_left": {"x": -4.90, "y": 0.89},
            "top_right": {"x": -3.27, "y": 0.89},
            "bottom_left": {"x": -4.90, "y": -3.23},
            "bottom_right": {"x": -3.27, "y": -3.23}
        },
        "main_pose": {
            "position": {"x": -3.273, "y": 0.894, "z": 0.0},
            "orientation": {"x": 0.0, "y": 0.0, "z": -0.9535, "w": 0.30135}
        }
    },
    "Kitchen": {
        "bounding_box": {
            "top_left": {"x": -5.15, "y": 5.44},
            "top_right": {"x": -3.36, "y": 5.44},
            "bottom_left": {"x": -5.15, "y": 1.40},
            "bottom_right": {"x": -3.36, "y": 1.40}
        },
        "main_pose": {
            "position": {"x": -3.362, "y": 5.262, "z": 0.0},
            "orientation": {"x": 0.0, "y": 0.0, "z": -0.9535, "w": 0.30135}
        }
    },
    "Bathroom": {
        "bounding_box": {
            "top_left": {"x": 2.49, "y": 5.36},
            "top_right": {"x": 3.96, "y": 5.36},
            "bottom_left": {"x": 2.49, "y": 1.67},
            "bottom_right": {"x": 3.96, "y": 1.67}
        },
        "main_pose": {
            "position": {"x": 2.576, "y": 5.087, "z": 0.0},
            "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 0.87761}
        }
    },
    "PartyHall": {
        "bounding_box": {
            "top_left": {"x": 4.91, "y": 5.24},
            "top_right": {"x": 8.87, "y": 5.24},
            "bottom_left": {"x": 4.91, "y": 0.60},
            "bottom_right": {"x": 8.87, "y": 0.60}
        },
        "main_pose": {
            "position": {"x": 8.867, "y": 4.991, "z": 0.0},
            "orientation": {"x": 0.0, "y": 0.0, "z": -0.84839, "w": 0.52937}
        }
    },
    "ConferenceRoom": {
        "bounding_box": {
            "top_left": {"x": 7.23, "y": 0.13},
            "top_right": {"x": 8.84, "y": 0.13},
            "bottom_left": {"x": 7.23, "y": -4.17},
            "bottom_right": {"x": 8.84, "y": -4.17}
        },
        "main_pose": {
            "position": {"x": 8.176, "y": 0.130, "z": 0.0},
            "orientation": {"x": 0.0, "y": 0.0, "z": -0.87659, "w": 0.48124}
        }
    }
}


def initialize_navigator():
    """Initializes the navigator, sets the initial pose, and waits for Nav2 activation."""
    navigator = BasicNavigator()

    # Wait for all lifecycle nodes (localizer, planner, controller, etc.) to become active.
    navigator.waitUntilNav2Active()
    navigator.get_logger().info('Nav2 is active, proceeding...')

    # Set the robot’s initial pose.
    initial_pose = PoseStamped()
    initial_pose.header.frame_id = 'map'
    initial_pose.header.stamp = navigator.get_clock().now().to_msg()
    initial_pose.pose.position.x = 0.0
    initial_pose.pose.position.y = 0.0
    initial_pose.pose.position.z = 0.0
    initial_pose.pose.orientation.x = 0.0
    initial_pose.pose.orientation.y = 0.0
    initial_pose.pose.orientation.z = 0.0
    initial_pose.pose.orientation.w = 1.0

    navigator.setInitialPose(initial_pose)
    navigator.get_logger().info('Initial pose has been set.')

    return navigator

if __name__ == "__main__":
    main()