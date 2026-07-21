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
        self.missed_marker_count = 0
        self.current_marker = None
        self.last_twist = Twist()
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
                self.missed_marker_count += 1
                if self.missed_marker_count < 10:
                    self.get_logger().info("Marker lost, but still within tolerance")
                    self.cmd_vel_publisher.publish(self.last_twist)
                    return
                self.get_logger().info("Marker lost, returning to SEARCHING state")
                self.state = MissionState.SEARCHING
                self.current_marker = None
            return
        self.current_marker = marker
        self.missed_marker_count = 0
        if self.state == MissionState.SEARCHING:
            self.get_logger().info(f"Marker found! ID: {marker['id']}, Center: ({marker['center_x']}, {marker['center_y']}), Width: {marker['width']}")
            self.cmd_vel_publisher.publish(Twist())  
            self.missed_marker_count = 0
            self.current_marker = marker
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
    
    def rotate_robot(self, angular_speed=0.15):
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
            if self.current_marker is None and self.missed_marker_count < 10:
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
            return
        self.get_logger().info("APPROACHING")
        twist = Twist()
        image_center = 320       
        error = self.current_marker["center_x"] - image_center
        self.get_logger().info(f"center={self.current_marker['center_x']} "f"error={error}")
        # First center the marker
        if abs(error) > 60 and self.current_marker["width"] < 60:
            if error < 0:
                twist.angular.z = 0.1   # marker left → turn left
            else:
                twist.angular.z = -0.1      # marker right → turn right
            twist.linear.x = 0.15 
        # Marker centered, move forward
        elif self.current_marker["width"] < 60:
            twist.linear.x = 0.15
        # Close enough
        else:
            self.cmd_vel_publisher.publish(Twist())   # Stop
            self.state = MissionState.LOGGING
            return
        self.get_logger().info(f"Publishing linear={twist.linear.x}, angular={twist.angular.z}")
        self.last_twist = twist
        self.cmd_vel_publisher.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    mission_node = MissionNode()
    rclpy.spin(mission_node)
    mission_node.destroy_node()
    rclpy.shutdown()
if __name__ == "__main__":
    main()
