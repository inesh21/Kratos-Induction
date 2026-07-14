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


class RobotState(Enum):
    NAVIGATING = 1
    SEARCHING = 2
    APPROACHING = 3
    BACKING_UP = 4


class MissionNode(Node):

    def __init__(self):
        super().__init__("mission_node")

        self.get_logger().info("Mission node started!")

        self.load_waypoints()

        self.nav_client = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self.get_logger().info("Waiting for Nav2...")
        self.nav_client.wait_for_server()
        self.get_logger().info("Connected to Nav2!")

        self.current_waypoint = 0
        self.state = RobotState.NAVIGATING

        self.bridge = CvBridge()
        self.detector = ArucoDetector()
        self.current_marker_id = None
        self.current_pose = None

        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.image_subscriber = self.create_subscription(
            Image, "/turtlebot/rgb", self.image_callback, 10
        )
        self.odom_subscriber = self.create_subscription(
            Odometry, "/odom", self.odom_callback, 10
        )

        self.search_timer = None
        self.nav_timer = None
        self.backup_timer = None
        self.goal_handle = None
        self.approach_lost_count = 0

        self.search_angular_speed = 0.18
        self.approach_linear_speed = 0.08
        self.approach_angular_gain = -0.0025
        self.max_approach_angular = 0.18
        self.finish_marker_width = 160
        self.max_lost_frames = 15
        self.nav_timeout = 60.0
        self.backup_speed = -0.15
        self.backup_duration = 2.0

        self.log_file_path = os.path.join(
            os.path.expanduser("~"), "detected_targets.txt"
        )

        self.send_goal(self.waypoints[self.current_waypoint])

    def load_waypoints(self):
        package_share = get_package_share_directory("aruco_search_mission")
        yaml_file = os.path.join(package_share, "config", "aruco_waypoints.yaml")

        self.get_logger().info(f"Loading YAML from: {yaml_file}")

        with open(yaml_file, "r") as file:
            data = yaml.safe_load(file)

        self.waypoints = data["waypoints"]
        self.get_logger().info(f"Loaded {len(self.waypoints)} waypoints")

        for i, wp in enumerate(self.waypoints):
            self.get_logger().info(
                f"Waypoint {i+1}: {wp['name']} "
                f"({wp['x']}, {wp['y']}, theta={wp['theta']}) "
                f"- {wp['hint']}"
            )

    def send_goal(self, waypoint):
        self.state = RobotState.NAVIGATING
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = "map"
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = float(waypoint["x"])
        goal_msg.pose.pose.position.y = float(waypoint["y"])
        goal_msg.pose.pose.position.z = 0.0
        goal_msg.pose.pose.orientation.w = 1.0

        self.get_logger().info(
            f"Sending goal to ({waypoint['x']}, {waypoint['y']})"
        )
        send_goal_future = self.nav_client.send_goal_async(
            goal_msg, feedback_callback=self.nav_feedback_callback
        )
        send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn("Goal rejected, starting search from current position")
            self.start_search()
            return

        self.goal_handle = goal_handle
        self.get_logger().info("Goal accepted")
        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(self.goal_result_callback)

        self.nav_timer = self.create_timer(
            self.nav_timeout, self.nav_timeout_callback
        )

    def nav_feedback_callback(self, feedback_msg):
        pass

    def nav_timeout_callback(self):
        self.cancel_nav_timer()
        self.get_logger().warn("Navigation timed out, cancelling goal and searching from here")
        if self.goal_handle is not None:
            self.goal_handle.cancel_goal_async()
            self.goal_handle = None
        self.start_search()

    def cancel_nav_timer(self):
        if self.nav_timer is not None:
            self.nav_timer.cancel()
            self.nav_timer = None

    def goal_result_callback(self, future):
        self.cancel_nav_timer()
        self.goal_handle = None
        result = future.result()
        if result.status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(f"Reached waypoint {self.current_waypoint + 1}")
            self.start_search()
        elif result.status == GoalStatus.STATUS_CANCELED:
            self.get_logger().info("Navigation goal was cancelled")
        else:
            self.get_logger().warn(
                f"Navigation failed (status {result.status}), searching from here"
            )
            self.start_search()

    def image_callback(self, msg):
        if self.state not in (RobotState.SEARCHING, RobotState.APPROACHING):
            return

        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            marker = self.detector.detect(frame)

            if self.state == RobotState.SEARCHING:
                if marker is None:
                    return
                self.current_marker_id = int(marker["id"])
                self.get_logger().info(f"Found Marker {self.current_marker_id}")
                self.stop_robot()
                self.state = RobotState.APPROACHING
                self.approach_lost_count = 0

            elif self.state == RobotState.APPROACHING:
                if marker is None:
                    self.approach_lost_count += 1
                    if self.approach_lost_count >= self.max_lost_frames:
                        self.get_logger().warn("Lost marker during approach, resuming search")
                        self.start_search()
                    return
                self.approach_lost_count = 0
                image_center = frame.shape[1] / 2
                self.approach_marker(marker, image_center)

        except Exception as e:
            self.get_logger().error(str(e))

    def start_search(self):
        self.get_logger().info("Starting visual search...")
        self.state = RobotState.SEARCHING
        self.current_marker_id = None
        if self.search_timer is None:
            self.search_timer = self.create_timer(0.1, self.rotate_robot)
        else:
            self.search_timer.reset()

    def rotate_robot(self):
        if self.state != RobotState.SEARCHING:
            return
        msg = Twist()
        msg.angular.z = self.search_angular_speed
        self.cmd_pub.publish(msg)

    def stop_robot(self):
        self.cmd_pub.publish(Twist())
        if self.search_timer is not None:
            self.search_timer.cancel()
            self.search_timer = None

    def approach_marker(self, marker, image_center):
        marker_width = marker["marker_width"]

        if marker_width >= self.finish_marker_width:
            self.finish_target()
            return

        error = marker["center_x"] - image_center
        msg = Twist()
        msg.angular.z = max(
            min(self.approach_angular_gain * error, self.max_approach_angular),
            -self.max_approach_angular,
        )

        if abs(error) < 80:
            msg.linear.x = self.approach_linear_speed

        self.cmd_pub.publish(msg)

    def finish_target(self):
        self.stop_robot()
        self.log_target()
        self.current_waypoint += 1

        if self.current_waypoint < len(self.waypoints):
            self.get_logger().info("Backing up before next waypoint...")
            self.start_backup()
        else:
            self.get_logger().info("All waypoints completed! Mission done.")

    def start_backup(self):
        self.state = RobotState.BACKING_UP
        msg = Twist()
        msg.linear.x = self.backup_speed
        self.cmd_pub.publish(msg)
        self.backup_timer = self.create_timer(
            self.backup_duration, self.backup_done
        )

    def backup_done(self):
        if self.backup_timer is not None:
            self.backup_timer.cancel()
            self.backup_timer = None
        self.stop_robot()
        self.get_logger().info("Backup complete, dispatching next waypoint...")
        self.send_goal(self.waypoints[self.current_waypoint])

    def odom_callback(self, msg):
        self.current_pose = msg.pose.pose

    def log_target(self):
        if self.current_pose is None:
            self.get_logger().warn("No odom pose available, cannot log position")
            return

        x = self.current_pose.position.x
        y = self.current_pose.position.y
        log_text = (
            f"Target found! ID: {self.current_marker_id}, "
            f"Rover Position: ({x:.2f}, {y:.2f})"
        )

        try:
            with open(self.log_file_path, "a") as f:
                f.write(log_text + "\n")
            self.get_logger().info(f"Logged: {log_text}")
        except Exception as e:
            self.get_logger().error(f"Failed to write log: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = MissionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
