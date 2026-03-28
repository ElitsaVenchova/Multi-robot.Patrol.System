import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math

class PatrolBot(Node):
    def __init__(self):
        super().__init__('patrol_bot')

        # Namespace parameter (e.g. tb3_1)
        self.declare_parameter('robot_ns', '')
        ns = self.get_parameter('robot_ns').get_parameter_value().string_value
        self.ns = ns if ns else 'tb3_1'

        self.cmd_topic = f'/{self.ns}/cmd_vel'
        self.odom_topic = f'/{self.ns}/odom'

        self.pub = self.create_publisher(Twist, self.cmd_topic, 10)
        self.create_subscription(Odometry, self.odom_topic, self.odom_callback, 10)

        # Border waypoints (square around world edges)
        self.waypoints = [
            (-1.8, -1.8), (1.8, -1.8),
            (1.8, 1.8), (-1.8, 1.8)
        ]

        # Shift start point per robot
        index = int(self.ns.split('_')[-1]) if '_' in self.ns else 0
        self.start_index = index % len(self.waypoints)
        self.current_wp = self.start_index
        self.pos = (0.0, 0.0)
        self.yaw = 0.0

        self.timer = self.create_timer(0.1, self.loop)
        self.get_logger().info(f'{self.ns} started patrol from waypoint {self.start_index}')

    def odom_callback(self, msg):
        self.pos = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        # Extract yaw from quaternion
        import math
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.yaw = math.atan2(siny_cosp, cosy_cosp)

    def loop(self):
        # Current goal
        gx, gy = self.waypoints[self.current_wp]
        dx, dy = gx - self.pos[0], gy - self.pos[1]
        dist = math.hypot(dx, dy)
        goal_angle = math.atan2(dy, dx)
        angle_error = self.normalize_angle(goal_angle - self.yaw)

        cmd = Twist()
        if dist > 0.1:
            cmd.linear.x = 0.15
            cmd.angular.z = 0.8 * angle_error
        else:
            self.current_wp = (self.current_wp + 1) % len(self.waypoints)
            self.get_logger().info(f'{self.ns} moving to waypoint {self.current_wp}')

        self.pub.publish(cmd)

    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

def main(args=None):
    rclpy.init(args=args)
    node = PatrolBot()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
