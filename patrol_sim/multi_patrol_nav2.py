#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
import math, time
import asyncio
from rclpy.parameter import Parameter

class PatrolNode(Node):
    def __init__(self):
        super().__init__('multi_patrol_nav2')
        self.declare_parameter('robot_ns', '')
        self.declare_parameter('waypoints', Parameter.Type.STRING_ARRAY)

        self.ns = self.get_parameter('robot_ns').get_parameter_value().string_value
        wp_list = self.get_parameter('waypoints').get_parameter_value().string_array_value

        # parse waypoints: expect strings "x,y,yaw"
        self.waypoints = []
        for s in wp_list:
            try:
                x,y,yaw = map(float, s.split(','))
                self.waypoints.append((x,y,yaw))
            except Exception:
                self.get_logger().warning(f'Bad waypoint entry: {s}')

        if not self.waypoints:
            self.get_logger().error('No waypoints provided, exiting')
            raise RuntimeError('No waypoints')

        action_name = f'/{self.ns}/navigate_to_pose'
        self._action_client = ActionClient(self, NavigateToPose, action_name)
        self.get_logger().info(f'Patrol node for {self.ns} -> action: {action_name}')

    def make_pose_stamped(self, x, y, yaw_deg=0.0):
        p = PoseStamped()
        p.header.frame_id = 'map'
        p.header.stamp = self.get_clock().now().to_msg()
        p.pose.position.x = float(x)
        p.pose.position.y = float(y)
        yaw = math.radians(yaw_deg)
        p.pose.orientation.z = math.sin(yaw/2.0)
        p.pose.orientation.w = math.cos(yaw/2.0)
        return p

    async def send_goal_async(self, pose):
        # Wait for server
        await self._action_client.wait_for_server()

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose

        # Send goal
        self.get_logger().info(f'{self.ns}: Goal sent.')
        goal_future = self._action_client.send_goal_async(goal_msg)
        goal_handle = await goal_future

        if not goal_handle.accepted:
            self.get_logger().warning(f'{self.ns}: Goal rejected.')
            return None

        # Wait for result
        result_future = goal_handle.get_result_async()
        result = await result_future

        return result.status


    def run_loop(self):
        loop = asyncio.get_event_loop()
        while rclpy.ok():
            for (x,y,yaw) in self.waypoints:
                pose = self.make_pose_stamped(x,y,yaw)
                self.get_logger().info(f'{self.ns}: sending goal ({x:.2f},{y:.2f},{yaw:.1f})')
                try:
                    status = loop.run_until_complete(self.send_goal_async(pose))
                    self.get_logger().info(f'{self.ns}: goal finished status {status}')
                except Exception as e:
                    self.get_logger().error(f'{self.ns}: goal error {e}')
                time.sleep(0.5)

def main(args=None):
    rclpy.init(args=args)
    node = PatrolNode()
    try:
        node.run_loop()
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
