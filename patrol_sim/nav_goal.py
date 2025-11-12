#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
import math
import time

class NavGoal(Node):
    def __init__(self):
        super().__init__('nav_goal')
        self._client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

    def make_pose(self, x, y, yaw_deg=0.0):
        msg = PoseStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x = x
        msg.pose.position.y = y
        yaw = math.radians(yaw_deg)
        msg.pose.orientation.z = math.sin(yaw/2.0)
        msg.pose.orientation.w = math.cos(yaw/2.0)
        return msg

    def send_goal(self, x, y, yaw_deg=0.0):
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = self.make_pose(x, y, yaw_deg)

        self._client.wait_for_server()
        self.get_logger().info(f'Sending goal to ({x}, {y})')
        future = self._client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future)
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected')
            return

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        self.get_logger().info(f'Result: {result_future.result().status}')

def main(args=None):
    rclpy.init(args=args)
    node = NavGoal()
    time.sleep(2)
    node.send_goal(1.5, 0.0, 0.0)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
