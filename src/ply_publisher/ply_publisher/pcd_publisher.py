#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
import numpy as np
import open3d as o3d
from PIL import Image, ImageDraw
import yaml
import matplotlib.pyplot as plt
from matplotlib import cm

class SimpleCostmapPublisher(Node):
    def __init__(self):
        super().__init__('simple_costmap_publisher')
        self.costmap_publisher = self.create_publisher(OccupancyGrid, '/costmap', 10)
        self.pcd_file = '/home/kage/Downloads/output.pcd'
        self.resolution = 1.0
        self.timer = self.create_timer(0.5, self.publish_costmap)
        self.get_logger().info('Loading PCD and creating colored elevation map...')
        self.create_costmaps_from_pcd()

    def create_costmaps_from_pcd(self):
        try:
            pcd = o3d.io.read_point_cloud(self.pcd_file)
            points = np.asarray(pcd.points)
            if len(points) == 0:
                self.get_logger().error('No points found in PCD file')
                return

            min_z = np.min(points[:, 2])
            max_z = np.max(points[:, 2])

            # Bin elevations per 1m
            bins = np.arange(min_z, max_z + 1, 1.0)
            counts, _ = np.histogram(points[:, 2], bins)

            # Sort bins by descending count
            sorted_indices = np.argsort(counts)[::-1]
            cmap = cm.get_cmap('tab20', len(counts))
            height_color_map = {i: sorted_indices.tolist().index(i) for i in range(len(counts))}

            # Compute grid size
            min_x, max_x = np.min(points[:, 0]), np.max(points[:, 0])
            min_y, max_y = np.min(points[:, 1]), np.max(points[:, 1])
            width = int((max_x - min_x) / self.resolution) + 1
            height = int((max_y - min_y) / self.resolution) + 1

            # Initialize RGB image array with gray background
            img_array = np.full((height, width, 3), 200, dtype=np.uint8)

            # Colorize each point
            for pt in points:
                xi = int((pt[0] - min_x) / self.resolution)
                yi = int((pt[1] - min_y) / self.resolution)
                if 0 <= xi < width and 0 <= yi < height:
                    bin_idx = int(pt[2] - min_z)
                    if bin_idx >= len(bins) - 1:
                        bin_idx = len(bins) - 2
                    color_idx = height_color_map.get(bin_idx, 0)
                    rgba = cmap(color_idx)
                    color = tuple(int(c * 255) for c in rgba[:3])
                    img_array[height - 1 - yi, xi] = color

            self.save_colored_map(img_array, min_x, min_y, bins, counts, sorted_indices)

        except Exception as e:
            self.get_logger().error(f'Failed to create elevation threshold map: {e}')

    def save_colored_map(self, img_array, origin_x, origin_y, bins, counts, sorted_indices):
        # Save the colored map
        img = Image.fromarray(img_array, mode='RGB')
        map_path = '/home/kage/elevation_threshold_map.png'
        img.save(map_path)

        # Create legend image
        legend_height = 20 * len(counts)
        legend = Image.new('RGB', (300, legend_height), (255, 255, 255))
        legend_draw = ImageDraw.Draw(legend)
        cmap = cm.get_cmap('tab20', len(counts))
        for i, idx in enumerate(sorted_indices):
            color = tuple(int(c * 255) for c in cmap(i)[:3])
            legend_draw.rectangle([(0, i * 20), (20, i * 20 + 20)], fill=color)
            label = f'{bins[idx]:.1f}m to {bins[idx + 1]:.1f}m: {counts[idx]} pts'
            legend_draw.text((25, i * 20), label, fill=(0, 0, 0))
        legend_path = '/home/kage/elevation_legend.png'
        legend.save(legend_path)

        # Save YAML
        yaml_content = {
            'image': 'elevation_threshold_map.png',
            'resolution': float(self.resolution),
            'origin': [float(origin_x), float(origin_y), 0.0],
            'negate': 0,
            'occupied_thresh': 0.65,
            'free_thresh': 0.25
        }
        yaml_path = '/home/kage/elevation_threshold_map.yaml'
        with open(yaml_path, 'w') as f:
            yaml.dump(yaml_content, f)

        self.get_logger().info(f'Saved colored elevation map to {map_path} and legend to {legend_path}')

    def publish_costmap(self):
        pass  # No ROS topic publishing for this map


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
