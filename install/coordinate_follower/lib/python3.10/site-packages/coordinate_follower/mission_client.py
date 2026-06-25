#!/usr/bin/env python3
import os
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateToPose

class MissionClient(Node):
    def __init__(self):
        super().__init__('mission_client')
        
        self.declare_parameter('waypoint_file', '')
        file_path = self.get_parameter('waypoint_file').get_parameter_value().string_value
        
        self.waypoints = []
        self.current_index = 0
        self.success_count = 0
        
        if not file_path:
            self.get_logger().error("No waypoint_file parameter provided! Exiting.")
            return
            
        # This calls the method below
        self.load_waypoints(file_path)
        
        self._action_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        
        self.get_logger().info("Waiting for Nav2 action server...")
        self._action_client.wait_for_server()
        
        self.get_logger().info("Nav2 action server detected! Initiating mission...")
        self.send_next_waypoint()

    # MAKE SURE THIS IS ALIGNED EXACTLY WITH THE OTHER def STATEMENTS
    def load_waypoints(self, file_path):
        """Reads and parses waypoints from a space-separated text file safely."""
        if not os.path.exists(file_path):
            self.get_logger().error(f"Waypoint file not found at: {file_path}")
            return

        try:
            with open(file_path, 'r') as f:
                for line in f:
                    line = line.strip()
                    if not line or line.startswith('#'):
                        continue
                    parts = line.split()
                    if len(parts) >= 2:
                        x = float(parts[0])
                        y = float(parts[1])
                        self.waypoints.append((x, y))
            self.get_logger().info(f"Loaded {len(self.waypoints)} waypoints from {file_path}")
        except Exception as e:
            self.get_logger().error(f"Failed to parse waypoint file: {str(e)}")

    def send_next_waypoint(self):
        if self.current_index >= len(self.waypoints):
            self.get_logger().info(
                f"Mission complete: {self.success_count}/{len(self.waypoints)} waypoints reached"
            )
            return

        x, y = self.waypoints[self.current_index]
        self.get_logger().info(
            f"Dispatching waypoint {self.current_index + 1}/{len(self.waypoints)}: x={x:.2f}, y={y:.2f}"
        )

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = "map"
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.orientation.w = 1.0

        send_goal_future = self._action_client.send_goal_async(
            goal_msg, 
            feedback_callback=self.feedback_callback
        )
        send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn(f"Waypoint {self.current_index + 1} was REJECTED by Nav2.")
            self.current_index += 1
            self.send_next_waypoint()
            return

        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(self.result_callback)

    def feedback_callback(self, feedback_msg):
        distance = feedback_msg.feedback.distance_remaining
        self.get_logger().info(f"Feedback: distance remaining = {distance:.2f} m")

    def result_callback(self, future):
        result = future.result()
        status = result.status

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(f"Waypoint {self.current_index + 1} SUCCEEDED")
            self.success_count += 1
        else:
            self.get_logger().warn(f"Waypoint {self.current_index + 1} FAILED or ABORTED with status code: {status}")

        self.current_index += 1
        self.send_next_waypoint()

def main(args=None):
    rclpy.init(args=args)
    node = MissionClient()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()