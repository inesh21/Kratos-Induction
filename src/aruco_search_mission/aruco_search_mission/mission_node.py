'''Induction Assignment 4: Autonomous Waypoint Navigation &

OpenCV ArUco Search
Project Kratos – Autonomous Subsystem

Mission Briefing
Welcome back to the Autonomous Subsystem of Project Kratos.
In the previous assignments, you drove the TurtleBot using manual teleoperation and basic geometry

scripts. Now, we are raising the stakes. The rover needs to navigate a hazardous environment au-
tonomously, locate scattered targets, and log their identities.

Your job this week: write a ROS 2 node that reads a series of rough search zone coordinates from
aruco_waypoints.yaml. Your node must use the Nav2 navigation stack to drive the rover to each
coordinates. Since these waypoints only represent rough coordinates, once the rover arrives at a waypoint,
your node must switch to a visual search routine (e.g., rotating or executing a scanning pattern). Using
the robot’s onboard RGB camera feed and OpenCV, you must find the nearby vertical ArUco marker
(a cube with marker texture on all sides), detect its ID, record the ID and the robot’s coordinates at the
moment of detection, and then proceed to the next waypoint.
The pipeline you are implementing:
Read YAML Waypoints –> Nav2 Go-to Goal –> Visual Search Rotation –> OpenCV ArUco Detection
–> Save Log (.txt)
Objective
The goal of this assignment is to develop an autonomous search-and-identify script that coordinates
high-level path planning (using Nav2) with local vision-based target acquisition (using OpenCV ArUco
detection) and file reporting.
Topics Covered
• Parsing configuration files in Python (pyyaml)
• ROS 2 Action Clients (nav2_msgs/action/NavigateToPose)
• Subscription to camera image topics (sensor_msgs/msg/Image)
• ROS 2 image processing with cv_bridge and OpenCV
• ArUco marker detection (cv2.aruco)
• State machine coordination / logical transitions in Python
Setup Requirements
Verify the following setup before writing your code:

SIMULATOR UPDATE: You must run git pull inside your local cloned genesis_sim di-
rectory to receive the latest updates. This contains the updated simulation configurations, 3D

ArUco marker assets, obstacle cubes, and the waypoints layout.
1. Ubuntu 22.04 LTS (or compatible Linux distribution)
2. ROS 2 Humble Hawksbill
3. Genesis Physics Engine (version 1.1.0 or newer)
4. Genesis ROS Bridge (genesis_ros) — built and sourced inside ~/ros2_ws
5. Genesis TurtleBot Simulator (genesis_sim) — updated via git pull
6. Python packages for OpenCV and YAML parsing:

Assignment 4: Autonomous Waypoint Navigation & OpenCV ArUco Search 1

Induction 2026 Project Kratos – Autonomous Subsystem

pip install opencv-python opencv-contrib-python pyyaml
Assignment Tasks
You are required to complete the following tasks:
1. YAML Parsing & Nav2 Navigation
Write a script that reads the waypoints coordinates from the aruco_waypoints.yaml file.
1a. YAML Parsing
• Read and parse the rough target positions (x, y) from aruco_waypoints.yaml.
• Print the coordinates of all target waypoints to the console on startup.
1b. Nav2 Action Client
• Initialize an Action Client for nav2_msgs/action/NavigateToPose.
• Send the coordinates to Nav2 to drive the robot to the first waypoint.
• Monitor feedback and wait until Nav2 reports that the robot has arrived at the goal pose before
triggering the search state.
2. Visual Search & OpenCV ArUco Detection
Once Nav2 arrives at a waypoint, the robot must search for the nearby ArUco tag.
2a. Visual Search Routine
• Implement a local searching state (for example, rotate the robot slowly by publishing low angular
velocities on /cmd_vel).
• The robot must scan the area until the onboard camera sees the ArUco marker.
2b. ArUco Marker Detection & Approaching
• Subscribe to the RGB camera feed on the topic /turtlebot/rgb.
• Use cv_bridge to convert the incoming ROS Image messages into OpenCV-compatible images.
• Detect ArUco markers using OpenCV’s ArUco module with the dictionary configuration DICT_4X4_1000
(Various IDs are placed in the scene).
• Once detected, drive the robot forward to stop exactly 0.25 meters away from the ArUco
tag (e.g., using visual servoing, estimated marker distance, or simple camera geometry metrics).
3. Data Logging & Execution
Record the findings of the mission.
3a. Save Target Logs
• For each discovered marker, capture its ID and the current (x, y) coordinates of the robot (read
from the /odom topic or /tf).
• Write this information in real-time to a file named detected_targets.txt. The file must append
each new target as it is found:
Target found! ID: 67, Rover Position: (2.48, 1.76)
Target found! ID: 69, Rover Position: (-1.95, 2.18)
Target found! ID: 88, Rover Position: (0.97, -2.44)
3b. Multi-Goal Sequencing
• Once the target is found and logged, transition back to the navigation state and dispatch the next
waypoint coordinates to Nav2.
• Repeat until all targets listed in the YAML configuration are found and logged.
'''

import os
import yaml
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from ament_index_python.packages import get_package_share_directory
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from aruco_search_mission.image_processor import ArucoDetector

from enum import Enum

class MissionState(Enum):
    NAVIGATING = 1
    SEARCHING = 2
    APPROACHING = 3
    LOGGING = 4


class MissionNode(Node):
    def __init__(self):
        super().__init__('aruco_search_mission')
        self.action_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_subscriber = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.image_subscriber = self.create_subscription(Image, '/turtlebot/rgb', self.image_callback, 10)
        self.bridge = CvBridge()
        self.aruco_detector = ArucoDetector()
        self.current_position = None
        self.state = MissionState.NAVIGATING
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.waypoints = []
        self.current_waypoint = 0
        self.current_marker = None
        self.load_waypoints(os.path.join(get_package_share_directory('aruco_search_mission'), 'config', 'aruco_waypoints.yaml'))
        if len(self.waypoints) > 0:
            wp = self.waypoints[self.current_waypoint]
            self.send_goal(wp["x"], wp["y"])
        self.timer=self.create_timer(0.1, self.timer_callback)


    def load_waypoints(self, yaml_file):
        with open(yaml_file, 'r') as file:
            waypoints = yaml.safe_load(file)
            self.waypoints = waypoints['waypoints']
            for wp in self.waypoints:
                self.get_logger().info(f"Loaded waypoint: {wp}")

    def send_goal(self, x, y):
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.orientation.w = 1.0  
        self.action_client.wait_for_server()
        self._send_goal_future = self.action_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle =future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return
        else:
            self.get_logger().info('Goal accepted :)')
            self._get_result_future = goal_handle.get_result_async()
            self._get_result_future.add_done_callback(self.get_result_callback)
    
    def image_callback(self, msg):
        self.get_logger().info("Image received")
        if self.state not in (
            MissionState.SEARCHING,
            MissionState.APPROACHING,
        ):
            return
        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        marker = self.aruco_detector.detect_markers(frame)
        self.get_logger().info(f"Marker = {marker}")
        if marker is None:
            if self.state ==MissionState.APPROACHING:
                self.current_marker = None
            return
        self.current_marker = marker
        if self.state == MissionState.SEARCHING:
            self.get_logger().info(f"Marker found! ID: {marker['id']}, Center: ({marker['center_x']}, {marker['center_y']}), Width: {marker['width']}")
            self.cmd_vel_publisher.publish(Twist())  
            self.state = MissionState.APPROACHING

    def get_result_callback(self, future):
        status = future.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('Goal succeeded!')
            self.current_marker = None
            self.state = MissionState.SEARCHING
        else:
            self.get_logger().info(f'Goal failed with status: {status}')
    
    def odom_callback(self, msg):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
    
    def rotate_robot(self, angular_speed=0.1):
        twist = Twist()
        twist.angular.z = angular_speed
        self.cmd_vel_publisher.publish(twist)

    def log_target(self, marker_id):
        with open('detected_targets.txt', 'a') as f:
            f.write(f"Target found! ID: {marker_id}, Rover Position: ({self.robot_x:.2f}, {self.robot_y:.2f})\n")
        self.get_logger().info(f"Logged target ID: {marker_id} at position ({self.robot_x:.2f}, {self.robot_y:.2f})")
    
    def timer_callback(self):
        self.get_logger().info(f"Current State: {self.state}")
        if self.state == MissionState.SEARCHING:
            if self.current_marker is None:
                self.rotate_robot()
        elif self.state == MissionState.APPROACHING:
            self.approach_marker()
        elif self.state == MissionState.LOGGING:
            self.log_target(self.current_marker["id"])
            self.current_marker = None
            stop= Twist()
            self.cmd_vel_publisher.publish(stop)
            self.current_waypoint += 1
            if self.current_waypoint < len(self.waypoints):
                wp = self.waypoints[self.current_waypoint]
                self.state = MissionState.NAVIGATING
                self.send_goal(wp["x"], wp["y"])
            else:
                self.get_logger().info("Mission Complete")
                self.current_marker = None


    def approach_marker(self):
        if self.current_marker is None:
            self.get_logger().info("No marker to approach")
            self.state = MissionState.SEARCHING
            return
        self.get_logger().info("APPROACHING")
        twist = Twist()
        image_center = 320       
        error = self.current_marker["center_x"] - image_center
        self.get_logger().info(f"center={self.current_marker['center_x']} "f"error={error}")
        # First center the marker
        if abs(error) > 30:
            if error < 0:
                twist.angular.z = 0.05   # marker left → turn left
            else:
                twist.angular.z = -0.05      # marker right → turn right
        # Marker centered, move forward
        elif self.current_marker["width"] < 100:
            twist.linear.x = 0.15
        # Close enough
        else:
            self.cmd_vel_publisher.publish(Twist())   # Stop
            self.state = MissionState.LOGGING
            return
        self.get_logger().info(f"Publishing linear={twist.linear.x}, angular={twist.angular.z}")
        self.cmd_vel_publisher.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    mission_node = MissionNode()
    rclpy.spin(mission_node)
    mission_node.destroy_node()
    rclpy.shutdown()
if __name__ == "__main__":
    main()
