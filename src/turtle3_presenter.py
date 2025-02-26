#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

visited_rooms = set()

def main(args=None):
    rclpy.init(args=args)
    navigator = initialize_navigator()

    # Optionally, start at a designated room (e.g. "FrontDoor").
    start_room = "FrontDoor"
    navigate_to_room(start_room, navigator)

    # Begin interactive tour.
    interactive_tour(navigator)


    print("This is it, hope you enjoyed!")
    # navigate_to_room(start_room, navigator)

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
    During the trip it continuously checks the current pose; if the robot's (x,y)
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
        print(f"Navigating to {room_name}")
        navigator.goThroughPoses(poses)
    else:
        print(f"Navigating to {room_name}")
        navigator.goToPose(goal_pose)

    announced_rooms = set()
    # Monitor the navigation until the task completes.
    while not navigator.isTaskComplete():
        rclpy.spin_once(navigator, timeout_sec=0.1)

        if navigator.current_pose is None:
            continue

        # Check each room to see if the robot is inside its bounding box.
        for rname, rdata in ROOMS.items():
            if rname == room_name:
                continue
            if rname not in announced_rooms and in_room(navigator.current_pose, rdata):
                print(f"Passing by {rname}...")
                announced_rooms.add(rname)

    result = navigator.getResult()
    if result == TaskResult.SUCCEEDED:
        print(f"This is the {room_name}!")
        visited_rooms.add(room_name)
        return True
    else:
        navigator.get_logger().error(f"Navigation to {room_name} failed with result: {result}")
        return False


def compute_path_length(path):
    """
    Computes the total Euclidean distance along a path.
    """
    length = 0.0
    poses = path.poses
    for i in range(len(poses) - 1):
        dx = poses[i+1].pose.position.x - poses[i].pose.position.x
        dy = poses[i+1].pose.position.y - poses[i].pose.position.y
        length += (dx*dx + dy*dy) ** 0.5
    return length

def calculate_best_room(navigator):
    """
    Calculates the best to visit next from the current position, 
    using the nav2 API to compute paths.
    """
    # Wait until the current pose is available.
    while navigator.current_pose is None:
        rclpy.spin_once(navigator, timeout_sec=0.1)

    # Wrap the current_pose (which is a Pose) into a PoseStamped.
    current_pose_stamped = PoseStamped()
    current_pose_stamped.header.frame_id = "map"
    current_pose_stamped.header.stamp = navigator.get_clock().now().to_msg()
    current_pose_stamped.pose = navigator.current_pose

    best_room = None
    best_path_length = float('inf')

    for room_name, room_data in ROOMS.items():
        if room_name in visited_rooms:
            continue

        # Build the goal pose from the room's main_pose.
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = "map"
        goal_pose.header.stamp = navigator.get_clock().now().to_msg()
        goal_pose.pose.position.x = room_data["main_pose"]["position"]["x"]
        goal_pose.pose.position.y = room_data["main_pose"]["position"]["y"]
        goal_pose.pose.position.z = room_data["main_pose"]["position"]["z"]
        goal_pose.pose.orientation.x = room_data["main_pose"]["orientation"]["x"]
        goal_pose.pose.orientation.y = room_data["main_pose"]["orientation"]["y"]
        goal_pose.pose.orientation.z = room_data["main_pose"]["orientation"]["z"]
        goal_pose.pose.orientation.w = room_data["main_pose"]["orientation"]["w"]

        # Retrieve the planned path using nav2.
        path = navigator.getPath(current_pose_stamped, goal_pose)
        if path is None:
            print(f"No valid path found to room {room_name}. Skipping this room.")
            continue

        path_length = compute_path_length(path)
        if path_length < best_path_length:
            best_path_length = path_length
            best_room = room_name

    return best_room



def full_tour(navigator):
    """
    Computes a full tour covering all non-visited rooms. For each remaining room, it:
      1. Calculates the best room to visit next using the computed path length.
      2. Commands the robot to navigate there.
      3. Waits for the guest to press Enter before continuing.
    """
    while len(visited_rooms) < len(ROOMS):
        next_room = calculate_best_room(navigator)
        if next_room is None:
            break

        print(f"Next best room to visit: {next_room}")
        if navigate_to_room(next_room, navigator):
            input("Press Enter to continue the tour...")
        else:
            navigator.get_logger().error(f"Failed to navigate to {next_room}. Retrying...")
    print("All rooms have been visited.")


def interactive_tour(navigator):
    """
    Runs an interactive loop that asks the user to specify the next room.
    - If the guest presses Enter without typing anything, the full tour is started.
    - If the guest inputs an invalid room name, available room names are printed and the user is asked again.
    - If the guest inputs a valid room, the robot navigates there.
    """
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
        print(f"Planning full tour for remaining rooms: {remaining}")
        full_tour(navigator)
    else:
        print("All rooms have been visited.")


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
    navigator.current_pose = None

    # Subscriber to amcl_pose topic.
    navigator.create_subscription(PoseWithCovarianceStamped, 'amcl_pose', 
                                  lambda msg: setattr(navigator, 'current_pose', msg.pose.pose), 10)
    return navigator

if __name__ == "__main__":
    main()
