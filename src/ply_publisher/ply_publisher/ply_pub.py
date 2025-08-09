#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
import numpy as np
import open3d as o3d
from sensor_msgs_py import point_cloud2

class PlyPublisher(Node):
    def __init__(self):
        super().__init__('ply_publisher')
        self.pub = self.create_publisher(PointCloud2, '/ply_cloud', 10)
        # Load once at startup:
        pcd = o3d.io.read_point_cloud('/home/kage/Downloads/output.ply')
        pts = np.asarray(pcd.points)
        # build PointCloud2 message
        header = Header(frame_id='map')
        self.pc2_msg = point_cloud2.create_cloud_xyz32(header, pts.tolist())
        # publish latched so Nav2 can grab it any time
        self.pub.publish(self.pc2_msg)
        self.get_logger().info(f"Published {len(pts)} points on ‘ply_cloud’")

def main(args=None):
    rclpy.init(args=args)
    node = PlyPublisher()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
