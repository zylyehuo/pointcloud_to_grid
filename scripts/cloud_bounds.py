#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
import math

class CloudBounds(Node):
    def __init__(self):
        super().__init__('cloud_bounds')
        self.sub = self.create_subscription(
            PointCloud2,
            '/rslidar_points',
            self.callback,
            10
        )

    def callback(self, msg):
        xs, ys, zs = [], [], []

        for p in point_cloud2.read_points(
            msg,
            field_names=('x', 'y', 'z'),
            skip_nans=True
        ):
            x, y, z = float(p[0]), float(p[1]), float(p[2])
            if math.isfinite(x) and math.isfinite(y) and math.isfinite(z):
                xs.append(x)
                ys.append(y)
                zs.append(z)

        if not xs:
            self.get_logger().warn('No valid points.')
            return

        print('frame:', msg.header.frame_id)
        print('points:', len(xs))
        print('x min/max:', min(xs), max(xs))
        print('y min/max:', min(ys), max(ys))
        print('z min/max:', min(zs), max(zs))
        print('suggest position_x:', (min(xs) + max(xs)) / 2.0)
        print('suggest position_y:', (min(ys) + max(ys)) / 2.0)
        print('suggest length_x:', max(xs) - min(xs) + 5.0)
        print('suggest length_y:', max(ys) - min(ys) + 5.0)

        rclpy.shutdown()

def main():
    rclpy.init()
    node = CloudBounds()
    rclpy.spin(node)

if __name__ == '__main__':
    main()

