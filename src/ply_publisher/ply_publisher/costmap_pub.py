#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
import numpy as np
import open3d as o3d
from PIL import Image
import yaml

class SimpleCostmapPublisher(Node):
    def __init__(self):
        super().__init__('simple_costmap_publisher')
        # Publisher for costmap
        self.costmap_publisher = self.create_publisher(OccupancyGrid, '/costmap', 10)

        # Parameters
        self.pcd_file = '/home/kage/Downloads/Point_clouds/autonomous_ground_2025_1.pcd'  # Your PCD file path
        self.relative_height_threshold = 5.0  # 10m above ground
        self.resolution = 1.0  # 1m per pixel

        # Timer to publish periodically
        self.timer = self.create_timer(0.5, self.publish_costmap)

        self.get_logger().info('Loading PCD and creating costmap...')
        self.create_costmap_from_pcd()

    def create_costmap_from_pcd(self):
        try:
            # Load point cloud
            pcd = o3d.io.read_point_cloud(self.pcd_file)
            points = np.asarray(pcd.points)

            if len(points) == 0:
                self.get_logger().error('No points found in PCD file')
                return

            # Compute ground and obstacle threshold
            min_z = np.min(points[:,2])  # Assuming ground level is around 1375m
            obstacle_thresh = min_z + self.relative_height_threshold

            # Filter obstacles
            obstacles = points[points[:, 2] > obstacle_thresh]

            # Compute XY bounds
            min_x, max_x = np.min(points[:,0]), np.max(points[:,0])
            min_y, max_y = np.min(points[:,1]), np.max(points[:,1])

            # Grid dimensions
            width = int((max_x - min_x) / self.resolution) + 1
            height = int((max_y - min_y) / self.resolution) + 1

            # Initialize grid: free=0, occupied=100
            grid = np.zeros((height, width), dtype=np.int8)

            # Mark obstacles
            for pt in obstacles:
                xi = int((pt[0] - min_x) / self.resolution)
                yi = int((pt[1] - min_y) / self.resolution)
                if 0 <= xi < width and 0 <= yi < height:
                    grid[yi, xi] = 100

            # Build OccupancyGrid message
            og = OccupancyGrid()
            og.header.frame_id = 'map'
            og.info.resolution = self.resolution
            og.info.width = width
            og.info.height = height
            og.info.origin.position.x = float(min_x)
            og.info.origin.position.y = float(min_y)
            og.info.origin.orientation.w = 1.0
            og.data = grid.flatten().tolist()

            self.occupancy_grid = og
            self.get_logger().info(f'Created costmap {width}x{height}')

            # Save map to disk
            self.save_map(grid, min_x, min_y)

        except Exception as e:
            self.get_logger().error(f'Failed to create costmap: {e}')

    def save_map(self, grid, origin_x, origin_y):
        # Convert grid to PGM format: occupied=0, free=254, unknown=205
        pgm = np.zeros_like(grid, dtype=np.uint8)
        pgm[grid == 100] = 0
        pgm[grid == 0] = 254

        # Save PGM
        pgm_img = Image.fromarray(np.flipud(pgm), mode='L')
        pgm_path = '/home/kage/global_costmap_map.pgm'
        pgm_img.save(pgm_path)
        self.get_logger().info(f'Saved PGM map to {pgm_path}')

        # Save YAML
        yaml_content = {
            'image': pgm_path.split('/')[-1],
            'resolution': float(self.resolution),
            'origin': [float(origin_x), float(origin_y), 0.0],
            'negate': 0,
            'occupied_thresh': 0.65,
            'free_thresh': 0.25
        }
        yaml_path = '/home/kage/global_costmap_map.yaml'
        with open(yaml_path, 'w') as f:
            yaml.dump(yaml_content, f)
        self.get_logger().info(f'Saved YAML map to {yaml_path}')

    def publish_costmap(self):
        if hasattr(self, 'occupancy_grid'):
            self.occupancy_grid.header.stamp = self.get_clock().now().to_msg()
            self.costmap_publisher.publish(self.occupancy_grid)
            self.get_logger().info('Published costmap')
        else:
            self.get_logger().warn('Costmap not ready yet')


def main(args=None):
    rclpy.init(args=args)
    node = SimpleCostmapPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
