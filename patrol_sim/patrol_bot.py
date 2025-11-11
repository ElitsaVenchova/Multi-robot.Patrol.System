#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
from math import radians
import time

class PatrolBot(Node):
    def __init__(self):
        super().__init__('patrol_bot')

        # Parameter for namespace
        self.declare_parameter('robot_ns', '')
        self.namespace = self.get_parameter('robot_ns').get_parameter_value().string_value
        
        # Publisher to move robot
        self.cmd_pub = self.create_publisher(Twist, f'/{self.namespace}/cmd_vel', 10)
        # Publisher to set initial pose
        self.pose_pub = self.create_publisher(
            PoseWithCovarianceStamped,
            f'{self.namespace}/initialpose',
            10
        )

        # Give Gazebo + nav stack time to start
        time.sleep(2)
        self.set_initial_pose(0.0, 0.0, 0.0)  # x, y, yaw(deg)

    def set_initial_pose(self, x, y, yaw_deg):
        """Publish initial global pose"""
        yaw = radians(yaw_deg)

        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = "map"
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.position.z = 0.0

        msg.pose.pose.orientation.z = yaw
        msg.pose.pose.orientation.w = 1.0  # simplified

        self.pose_pub.publish(msg)
        self.get_logger().info(f"{self.namespace}: initial pose set to x={x}, y={y}, yaw={yaw_deg}°")
        time.sleep(1)

    def move(self, speed=0.15, duration=2.0):
        """Move forward for `duration` seconds"""
        twist = Twist()
        twist.linear.x = speed

        start = time.time()
        while time.time() - start < duration:
            self.cmd_pub.publish(twist)
            time.sleep(0.1)

    def move_back_and_forth(self):
        """Infinite patrol loop"""
        self.get_logger().info(f"{self.namespace} starting back-and-forth patrol... Ctrl+C to stop.")

        while rclpy.ok():
            # Forward
            self.get_logger().info(f"{self.namespace} forward")
            self.move(speed=0.15, duration=2.0)

            # Stop briefly
            self.cmd_pub.publish(Twist())
            time.sleep(1)

            # Backward
            self.get_logger().info(f"{self.namespace} backward")
            self.move(speed=-0.15, duration=2.0)

            # Stop briefly
            self.cmd_pub.publish(Twist())
            time.sleep(1)


def main(args=None):
    rclpy.init(args=args)
    node = PatrolBot()
    
    try:
        node.move_back_and_forth()
    except KeyboardInterrupt:
        pass

    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
