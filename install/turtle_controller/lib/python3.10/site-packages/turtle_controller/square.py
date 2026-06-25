#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class Drawsquare(Node):
    def __init__(self):
        super().__init__('draw_square')
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer = self.create_timer(0.5,self.send_velocity_command)
        self.get_logger().info('Draw Square Node has been started')
        
    def go_straight(self):
        msg = Twist()
        msg.linear.x = 0.2
        msg.angular.z = 0.0
        self.cmd_vel_pub.publish(msg)
    def turn(self):
        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = 1.07
        self.cmd_vel_pub.publish(msg)
    def send_velocity_command(self):
        for _ in range(4):
            self.go_straight()
            rclpy.spin_once(self, timeout_sec=2.0)
            self.turn()
            rclpy.spin_once(self, timeout_sec=1.0)



def main(args=None):
    rclpy.init(args=args)
    node = Drawsquare()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()