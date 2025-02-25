#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from tf2_ros import TransformListener, Buffer

def main(args=None):
    rclpy.init(args=args)
    navigator = initialize_navigator()

    # Optionally, start at a designated room (e.g. "FrontDoor").
    start_room = "FrontDoor"
    navigate_to_room(start_room, navigator)

    # Begin interactive tour.
    interactive_tour(navigator)

    navigator.destroy_node()
    rclpy.shutdown()


def in_room(pose, room_data):
    """
    Returns True if the robot's (x,y) from the provided Pose is within
    the room's bounding box using:
      x_room_min < x_robot < x_room_max and y_room_min < y_robot < y_room_max.
    """
    x = pose.position.x
    y = pose.position.y
    bbox = room_data["bounding_box"]
    x_min = bbox["top_left"]["x"]
    x_max = bbox["top_right"]["x"]
    y_max = bbox["top_left"]["y"]
    y_min = bbox["bottom_left"]["y"]
    return (x_min < x < x_max) and (y_min < y < y_max)


def navigate_to_room(room_name, navigator):
    """
    Commands the robot to drive to the specified room.
    During the trip it continuously checks the current pose; if the robot’s (x,y)
    enters any room's bounding box (other than the target) and it has not already been
    announced, it logs that it is "Passing by" that room.
    
    For the Bedroom, an extra waypoint is inserted to avoid getting too close to a wall.
    """
    if room_name not in ROOMS:
        navigator.get_logger().error(f"Room '{room_name}' not found.")
        return False

    target_room = ROOMS[room_name]
    # Create the final goal pose from the room's main_pose.
    goal_pose = PoseStamped()
    goal_pose.header.frame_id = "map"
    goal_pose.header.stamp = navigator.get_clock().now().to_msg()
    goal_pose.pose.position.x = target_room["main_pose"]["position"]["x"]
    goal_pose.pose.position.y = target_room["main_pose"]["position"]["y"]
    goal_pose.pose.position.z = target_room["main_pose"]["position"]["z"]
    goal_pose.pose.orientation.x = target_room["main_pose"]["orientation"]["x"]
    goal_pose.pose.orientation.y = target_room["main_pose"]["orientation"]["y"]
    goal_pose.pose.orientation.z = target_room["main_pose"]["orientation"]["z"]
    goal_pose.pose.orientation.w = target_room["main_pose"]["orientation"]["w"]

    # For Bedroom, insert an extra waypoint.
    if room_name == "Bedroom":
        waypoint = PoseStamped()
        waypoint.header.frame_id = "map"
        waypoint.header.stamp = navigator.get_clock().now().to_msg()
        waypoint.pose.position.x = -3.9
        waypoint.pose.position.y = 3.6
        waypoint.pose.position.z = 0.0
        waypoint.pose.orientation.x = 0.0
        waypoint.pose.orientation.y = 0.0
        waypoint.pose.orientation.z = 0.0
        waypoint.pose.orientation.w = 1.0

        poses = [waypoint, goal_pose]
        navigator.get_logger().info(f"Navigating to {room_name} using an intermediate waypoint via goThroughPoses...")
        navigator.goThroughPoses(poses)
    else:
        navigator.get_logger().info(f"Navigating to {room_name} using goToPose...")
        navigator.goToPose(goal_pose)

    announced_rooms = set()
    # Monitor the navigation until the task completes.
    while not navigator.isTaskComplete():
        rclpy.spin_once(navigator, timeout_sec=0.1)

        if navigator.current_pose is None:
            navigator.get_logger().warn("Current pose is not yet available, skipping room check.")
            continue

        # Check each room (except the target) to see if the robot is inside its bounding box.
        for rname, rdata in ROOMS.items():
            if rname == room_name:
                continue
            if rname not in announced_rooms and in_room(navigator.current_pose, rdata):
                navigator.get_logger().info(f"Passing by {rname}...")
                announced_rooms.add(rname)

    result = navigator.getResult()
    if result == TaskResult.SUCCEEDED:
        navigator.get_logger().info(f"Arrived at {room_name}!")
        return True
    else:
        navigator.get_logger().error(f"Navigation to {room_name} failed with result: {result}")
        return False


def full_tour(navigator, visited_rooms):
    """
    Computes a full tour covering all non-visited rooms.
    First, it asks the user if the full tour should be completed automatically.
    If yes, the ordering of the remaining rooms is computed.
    Otherwise, the user is allowed to select specific rooms.
    """
    


def interactive_tour(navigator):
    """
    Runs an interactive loop that asks the user to specify the next room.
    - If the guest presses Enter without typing anything, the full tour (as defined in full_tour) is started.
    - If the guest inputs an invalid room name, available room names are printed and the user is asked again.
    - If the guest inputs a valid, not-yet-visited room, the robot navigates there.
    """
    visited_rooms = {"FrontDoor"}  # Assuming starting room was FrontDoor

    while len(visited_rooms) < len(ROOMS):
        cmd = input("Enter a specific room to visit (or press Enter for full tour): ").strip()

        if cmd == "":
            break

        if cmd not in ROOMS:
            print("Room name not recognized. Available rooms are:")
            for room in ROOMS.keys():
                print(f"  - {room}")
            continue

        if navigate_to_room(cmd, navigator):
            visited_rooms.add(cmd)
            resp = input("Would you like to visit another specific room? (y/n): ").strip().lower()
            if resp != 'y':
                break
        else:
            navigator.get_logger().error(f"Navigation to {cmd} failed. Please try again.")

    # After specific selections, perform full tour for any remaining rooms.
    remaining = set(ROOMS.keys()) - visited_rooms
    if remaining:
        navigator.get_logger().info(f"Planning full tour for remaining rooms: {remaining}")
        full_tour(navigator, visited_rooms)
    else:
        navigator.get_logger().info("All rooms have been visited.")


# Define the room annotations.
#   top_left  = (min_x, max_y)
#   top_right = (max_x, max_y)
#   bottom_left  = (min_x, min_y)
#   bottom_right = (max_x, min_y)

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
            "top_left": {"x": -5.10, "y": 0.90},
            "top_right": {"x": -3.27, "y": 0.90},
            "bottom_left": {"x": -5.10, "y": -3.23},
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
            "top_left": {"x": 9.10, "y": -0.12},
            "top_right": {"x": 9.10, "y": -4.63},
            "bottom_left": {"x": 7.22, "y": -0.12},
            "bottom_right": {"x": 7.22, "y": -4.63}
        },
        "main_pose": {
            "position": {"x": 8.96, "y": -0.230, "z": 0.0},
            "orientation": {"x": 0.0, "y": 0.0, "z": -0.87659, "w": 0.48124}
        }
    }
}

def initialize_navigator():
    navigator = BasicNavigator()
    navigator.waitUntilNav2Active()
    navigator.get_logger().info("Nav2 is active, proceeding...")

    # Set the robot’s initial pose.
    initial_pose = PoseStamped()
    initial_pose.header.frame_id = "map"
    initial_pose.header.stamp = navigator.get_clock().now().to_msg()
    initial_pose.pose.position.x = 0.0
    initial_pose.pose.position.y = 0.0
    initial_pose.pose.position.z = 0.0
    initial_pose.pose.orientation.x = 0.0
    initial_pose.pose.orientation.y = 0.0
    initial_pose.pose.orientation.z = 0.0
    initial_pose.pose.orientation.w = 1.0

    navigator.setInitialPose(initial_pose)
    navigator.get_logger().info("Initial pose has been set.")

    # Initialize current_pose as None.
    navigator.current_pose = None

    # Subscriber to amcl_pose topic.
    navigator.create_subscription(
        PoseWithCovarianceStamped,
        'amcl_pose',
        lambda msg: setattr(navigator, 'current_pose', msg.pose.pose),
        10
    )

    return navigator

if __name__ == "__main__":
    main()
