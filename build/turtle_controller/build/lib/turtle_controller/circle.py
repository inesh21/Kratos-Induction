#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class Drawcircle(Node):
    def __init__(self):
        super().__init__('draw_circle')
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer = self.create_timer(0.5,self.send_velocity_command)
        self.get_logger().info('Draw Circle Node has been started')
        
    def send_velocity_command(self):
        msg=Twist()
        msg.linear.x=0.2
        msg.angular.z=0.5
        self.cmd_vel_pub.publish(msg)



def main(args=None):
    rclpy.init(args=args)
    node = Drawcircle()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()