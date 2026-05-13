#!/usr/bin/env python3
import math
import numpy as np
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from puzzlebot_localization.map_loader import MapLoader


def euler_from_quaternion(q):
    x, y, z, w = q
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)
    sinp = 2.0 * (w * y - z * x)
    if abs(sinp) >= 1.0:
        pitch = math.copysign(math.pi / 2, sinp)
    else:
        pitch = math.asin(sinp)
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return [roll, pitch, yaw]


class FakeScanPublisher(Node):
    def __init__(self):
        super().__init__('fake_scan_publisher')
        self.declare_parameter('map_yaml_path', 'src/puzzlebot_localization/maps/obstacles.yaml')
        self.declare_parameter('range_max', 12.0)
        self.declare_parameter('range_min', 0.12)
        self.declare_parameter('angle_min', 0.0)
        self.declare_parameter('angle_max', 6.28318)
        self.declare_parameter('samples', 360)
        self.declare_parameter('noise_stddev', 0.0)

        map_yaml = self.get_parameter('map_yaml_path').get_parameter_value().string_value
        self.map_loader = MapLoader(map_yaml)
        self.range_max = self.get_parameter('range_max').get_parameter_value().double_value
        self.range_min = self.get_parameter('range_min').get_parameter_value().double_value
        self.angle_min = self.get_parameter('angle_min').get_parameter_value().double_value
        self.angle_max = self.get_parameter('angle_max').get_parameter_value().double_value
        self.samples = self.get_parameter('samples').get_parameter_value().integer_value
        self.noise_stddev = self.get_parameter('noise_stddev').get_parameter_value().double_value

        self.odom_sub = self.create_subscription(Odometry, '/puzzlebot_controller/odom', self.odom_cb, 10)
        self.scan_pub = self.create_publisher(LaserScan, '/scan', 10)
        self.timer = self.create_timer(0.1, self.publish_scan)

        self.robot_pose = None
        self.get_logger().info('FakeScanPublisher iniciado')

    def odom_cb(self, msg):
        q = msg.pose.pose.orientation
        theta = euler_from_quaternion([q.x, q.y, q.z, q.w])[2]
        self.robot_pose = (msg.pose.pose.position.x, msg.pose.pose.position.y, theta)

    def publish_scan(self):
        if self.robot_pose is None:
            return

        x, y, theta = self.robot_pose
        scan = LaserScan()
        scan.header.stamp = self.get_clock().now().to_msg()
        scan.header.frame_id = 'lidar_link'
        scan.angle_min = self.angle_min
        scan.angle_max = self.angle_max
        scan.angle_increment = (self.angle_max - self.angle_min) / self.samples
        scan.range_min = self.range_min
        scan.range_max = self.range_max
        scan.ranges = []

        step_size = self.map_loader.resolution  # 0.05 m steps for maximum accuracy
        for i in range(self.samples):
            angle = theta + self.angle_min + i * scan.angle_increment
            r = self.range_max
            d = step_size
            while d <= self.range_max:
                rx = x + d * math.cos(angle)
                ry = y + d * math.sin(angle)
                row, col = self.map_loader.world_to_map(rx, ry)
                if row < 0 or row >= self.map_loader.height or col < 0 or col >= self.map_loader.width:
                    r = d
                    break
                if self.map_loader.occupancy[row, col] == 100:
                    r = d
                    break
                d += step_size
            if self.noise_stddev > 0.0:
                r += np.random.normal(0.0, self.noise_stddev)
                r = max(self.range_min, min(r, self.range_max))
            scan.ranges.append(float(r))

        self.scan_pub.publish(scan)


def main():
    rclpy.init()
    node = FakeScanPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
