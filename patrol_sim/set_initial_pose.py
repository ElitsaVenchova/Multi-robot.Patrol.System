#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
import math
import time

class SetInitialPose(Node):
    def __init__(self):
        super().__init__('set_initial_pose')

        # Declare namespace parameter (for multi-robot use)
        self.declare_parameter('robot_ns', '')
        ns = self.get_parameter('robot_ns').get_parameter_value().string_value
        ns_prefix = f'/{ns}' if ns else ''

        # Publisher to /<ns>/initialpose  (RViz sends to this)
        topic = f'{ns_prefix}/initialpose'
        self.pub = self.create_publisher(PoseWithCovarianceStamped, topic, 10)

        # Define initial positions per robot (adjust for your Gazebo layout)
        poses = {
            'TB3_1': (0.0, 0.0, 0.0),
            'TB3_2': (2.0, 0.0, 0.0),
            'TB3_3': (0.0, 2.0, 90.0),
            'TB3_4': (-2.0, 0.0, 180.0)
        }

        # Pick the pose for this namespace, or default to origin
        x, y, yaw_deg = poses.get(ns, (0.0, 0.0, 0.0))
        yaw = math.radians(yaw_deg)

        # Build the message
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
        msg.pose.pose.orientation.w = math.cos(yaw / 2.0)
        msg.pose.covariance[0] = 0.25
        msg.pose.covariance[7] = 0.25
        msg.pose.covariance[35] = 0.0685

        # Wait a bit so AMCL is ready
        time.sleep(2.0)
        self.get_logger().info(f'[{ns or "global"}] publishing initial pose at ({x:.1f}, {y:.1f})')
        self.pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = SetInitialPose()
    rclpy.spin_once(node, timeout_sec=1.0)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
