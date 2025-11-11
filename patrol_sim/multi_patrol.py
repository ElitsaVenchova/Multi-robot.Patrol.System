#!/usr/bin/env python3
import rclpy, time, math, random
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped

class PatrolBot(Node):
    def __init__(self, name, init_pose, charge_pose):
        super().__init__(f'{name}_node')

        self.name = name
        self.init_pose = init_pose
        self.charge_pose = charge_pose
        
        self.cmd_pub  = self.create_publisher(Twist, f'/{name}/cmd_vel', 10)
        self.pose_pub = self.create_publisher(PoseWithCovarianceStamped, f'/{name}/initialpose', 10)

        self.battery = 100            # percentage
        self.state = "PATROL"         # PATROL → GO_CHARGE → CHARGING → PATROL

        time.sleep(1)
        self.set_initial_pose(*init_pose)
        self.get_logger().info(f"{name}: Ready.")

    def set_initial_pose(self, x, y, yaw_deg):
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = "map"
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        yaw = math.radians(yaw_deg)
        msg.pose.pose.orientation.z = math.sin(yaw / 2)
        msg.pose.pose.orientation.w = math.cos(yaw / 2)
        self.pose_pub.publish(msg)

    def move(self, v, duration):
        twist = Twist()
        twist.linear.x = v
        start = time.time()
        while time.time() - start < duration:
            self.cmd_pub.publish(twist)
            time.sleep(0.05)

    def stop(self):
        self.cmd_pub.publish(Twist())
        time.sleep(0.2)

    def go_to(self, x, y):
        """Very simple straight-line drive toward target coord"""
        self.get_logger().info(f"{self.name}: Going to charger...")
        
        # crude proportional control
        for _ in range(50):
            # simulate robot moving closer (ideal)
            self.move(0.15, 0.1)

        self.stop()

    def update_battery(self, delta):
        self.battery = max(0, min(100, self.battery + delta))

    def patrol_step(self):
        self.get_logger().info(f"{self.name}: PATROLLING battery={self.battery}%")
        
        # back & forth
        self.move(0.15, 1.8)
        self.stop()
        time.sleep(0.3)
        self.move(-0.15, 1.8)
        self.stop()
        time.sleep(0.3)

        self.update_battery(-random.uniform(3, 8))  # consumption

    def charging_step(self):
        self.get_logger().info(f"{self.name}: CHARGING battery={self.battery}%")
        self.update_battery(+10)
        time.sleep(1)

    def step(self):
        # FSM
        if self.state == "PATROL":
            if self.battery < 25:
                self.state = "GO_CHARGE"
                self.get_logger().info(f"{self.name}: Battery low, going to charge")
            else:
                self.patrol_step()

        elif self.state == "GO_CHARGE":
            # go to charger pose
            self.go_to(*self.charge_pose[:2])
            self.state = "CHARGING"

        elif self.state == "CHARGING":
            if self.battery >= 100:
                self.state = "PATROL"
                self.get_logger().info(f"{self.name}: Fully charged, resuming patrol")
            else:
                self.charging_step()

def main(args=None):
    rclpy.init(args=args)

    # (x, y, yaw) for initial robot poses
    init_poses = {
        "robot1": (0.0, 0.0,   0.0),
        "robot2": (1.2, 0.0,   180.0),
        # add more robots if needed
    }

    charge_station_pose = (0.0, 0.0, 0.0)  # middle of map

    bots = []
    for name, pose in init_poses.items():
        bots.append(PatrolBot(name, pose, charge_station_pose))

    try:
        while rclpy.ok():
            for b in bots:
                b.step()
    except KeyboardInterrupt:
        pass

    for b in bots:
        b.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
