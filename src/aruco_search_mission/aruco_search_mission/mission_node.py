import os
from unittest import result
import yaml

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from ament_index_python.packages import get_package_share_directory
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from aruco_search_mission.image_processor import ArucoDetector
from geometry_msgs.msg import Twist

from enum import Enum

class RobotState(Enum):
    NAVIGATING = 1
    SEARCHING = 2
    APPROACHING = 3
    LOGGING = 4


class MissionNode(Node):

    def __init__(self):
        super().__init__("mission_node")

        self.get_logger().info("Mission node started!")

        self.load_waypoints()
        self.nav_client = ActionClient(self,NavigateToPose,"navigate_to_pose")
        self.get_logger().info("Waiting for Nav2...")
        self.nav_client.wait_for_server()
        self.get_logger().info("Connected to Nav2!")
        self.current_waypoint = 0
        self.send_goal(self.waypoints[self.current_waypoint])

        self.bridge = CvBridge()
        self.image_recieved =False
        self.image_subscriber = self.create_subscription(Image,"turtlebot/rgb",self.image_callback,10)

        self.detector = ArucoDetector()
        self.marker_found = False
        self.current_marker_id = None
        self.state = RobotState.NAVIGATING

        self.cmd_pub = self.create_publisher(Twist,"/cmd_vel",10)
        self.search_timer = None


    def load_waypoints(self):

        package_share = get_package_share_directory("aruco_search_mission")

        yaml_file = os.path.join(
            package_share,
            "config",
            "aruco_waypoints.yaml"
        )

        self.get_logger().info(f"Loading YAML from:\n{yaml_file}")

        with open(yaml_file, "r") as file:
            data = yaml.safe_load(file)

        self.waypoints = data["waypoints"]

        self.get_logger().info(
            f"Loaded {len(self.waypoints)} waypoints\n"
        )

        for i, waypoint in enumerate(self.waypoints):

            self.get_logger().info(
                f"""
                    Waypoint {i+1}
                    Name  : {waypoint['name']}
                    X     : {waypoint['x']}
                    Y     : {waypoint['y']}
                    Theta : {waypoint['theta']}
                    Hint  : {waypoint['hint']}
                    """
                                )
    def send_goal(self,waypoint):
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = "map"
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = float(waypoint["x"])
        goal_msg.pose.pose.position.y = float(waypoint["y"])
        goal_msg.pose.pose.position.z = 0.0

        goal_msg.pose.pose.orientation.w = 1.0

        self.get_logger().info(f"Sending goal to ({waypoint['x']}, {waypoint['y']})")
        send_goal_future = self.nav_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.goal_response_callback)
    
    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info("Goal rejected")
            return

        self.get_logger().info("Goal accepted")
        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(self.goal_result_callback)
    
    def goal_result_callback(self, future):
        result = future.result()
        if result.status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(f"Reached waypoint {self.current_waypoint + 1}")
            self.start_search()

        else:
            self.get_logger().info(f"Navigation failed: {result.status}")
            
    def image_callback(self, msg):
        if self.state != RobotState.SEARCHING:
            return
        try:
            frame = self.bridge.imgmsg_to_cv2(msg,desired_encoding="bgr8")
            ids, corners = self.detector.detect(frame)
            if ids is None:
                return
            marker_id = int(ids[0][0])
            if self.marker_found:
                return
            self.marker_found = True
            self.current_marker_id = marker_id
            self.get_logger().info(f"Found Marker {marker_id}")
            self.stop_robot()
            self.state = RobotState.APPROACHING

            self.get_logger().info("Ready to approach marker...")

        except Exception as e:
            self.get_logger().error(str(e))

    def start_search(self):
        self.get_logger().info("Starting visual search...")
        self.state = RobotState.SEARCHING
        self.marker_found = False
        self.current_marker_id = None
        if self.search_timer is None:
                self.search_timer = self.create_timer(0.1,self.rotate_robot)

    def rotate_robot(self):
        msg = Twist()
        msg.angular.z = 0.35
        self.cmd_pub.publish(msg)

    def stop_robot(self):
        msg = Twist()
        self.cmd_pub.publish(msg)
        if self.search_timer is not None:
            self.search_timer.cancel()
            self.search_timer.cancel()


def main(args=None):

    rclpy.init(args=args)

    node = MissionNode()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == "__main__":
    main()