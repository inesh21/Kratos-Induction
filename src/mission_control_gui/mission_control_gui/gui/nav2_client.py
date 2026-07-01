"""ROS 2 node that dispatches waypoints to Nav2 one at a time."""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus


class MissionControlNode(Node):
    def __init__(self, log_callback):
        super().__init__('mission_control_gui')
        self.log = log_callback
        self.action_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')

        self.waypoints = []
        self.current_index = 0
        self.success_count = 0
        self.on_done = None

    def dispatch_waypoints(self, waypoints, on_done):
        self.waypoints = waypoints
        self.current_index = 0
        self.success_count = 0
        self.on_done = on_done

        self.log("Waiting for Nav2 action server...")
        if not self.action_client.wait_for_server(timeout_sec=5.0):
            self.log("ERROR: Nav2 action server not available.")
            self.on_done(self.success_count, len(self.waypoints))
            return

        self.send_next_waypoint()

    def send_next_waypoint(self):
        if self.current_index >= len(self.waypoints):
            self.on_done(self.success_count, len(self.waypoints))
            return

        x, y = self.waypoints[self.current_index]
        self.log(f"Dispatching waypoint {self.current_index + 1}/{len(self.waypoints)}: x={x:.2f}, y={y:.2f}")

        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        goal.pose.pose.orientation.w = 1.0

        future = self.action_client.send_goal_async(goal, feedback_callback=self.on_feedback)
        future.add_done_callback(self.on_goal_response)

    def on_feedback(self, feedback_msg):
        distance = feedback_msg.feedback.distance_remaining
        self.log(f"Feedback: distance remaining = {distance:.2f} m")

    def on_goal_response(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.log(f"Waypoint {self.current_index + 1} REJECTED")
            self.current_index += 1
            self.send_next_waypoint()
            return

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.on_result)

    def on_result(self, future):
        status = future.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.log(f"Waypoint {self.current_index + 1} SUCCEEDED")
            self.success_count += 1
        else:
            self.log(f"Waypoint {self.current_index + 1} FAILED (status={status})")

        self.current_index += 1
        self.send_next_waypoint()