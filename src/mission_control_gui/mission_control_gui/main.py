"""Entry point: launches the GUI and spins the ROS 2 node together via QTimer."""

import sys
import math

import rclpy
from rclpy.executors import SingleThreadedExecutor
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

from PyQt5.QtWidgets import QApplication
from PyQt5.QtCore import QTimer

from mission_control_gui.gui.main_window import MainWindow
from mission_control_gui.gui.nav2_client import MissionControlNode


def main():
    rclpy.init()
    app = QApplication(sys.argv)

    node = MissionControlNode(log_callback=lambda msg: None)  # replaced below
    estop_publisher = node.create_publisher(Twist, '/cmd_vel', 10)

    def publish_estop():
        node.get_logger().info("EMERGENCY STOP triggered")
        node.cancel_current_goal()
        estop_publisher.publish(Twist())
        window.log("EMERGENCY STOP triggered — goal cancelled, zero velocity published.")

    window = MainWindow(
        dispatch_callback=lambda waypoints: node.dispatch_waypoints(waypoints, window.mission_finished),
        estop_callback=publish_estop
    )
    node.log = window.log  # route ROS logs into the GUI log panel

    def odom_callback(msg):
        window.update_position(msg.pose.pose.position.x, msg.pose.pose.position.y)
        window.update_velocity(msg.twist.twist.linear.x, msg.twist.twist.angular.z)

    def scan_callback(msg):
        valid_ranges = [r for r in msg.ranges if r > 0.0 and not math.isinf(r)]
        if valid_ranges:
            window.update_lidar(min(valid_ranges))

    node.create_subscription(Odometry, '/odom', odom_callback, 10)
    node.create_subscription(LaserScan, '/turtlebot/scan', scan_callback, 10)

    executor = SingleThreadedExecutor()
    executor.add_node(node)

    timer = QTimer()
    timer.timeout.connect(lambda: executor.spin_once(timeout_sec=0))
    timer.start(20)  # ~50 Hz

    window.show()
    exit_code = app.exec_()

    node.destroy_node()
    rclpy.shutdown()
    sys.exit(exit_code)


if __name__ == '__main__':
    main()