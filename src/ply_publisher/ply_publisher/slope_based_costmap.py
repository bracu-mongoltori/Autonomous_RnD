#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
import numpy as np
import open3d as o3d
from PIL import Image
import yaml
from scipy import ndimage
import matplotlib.pyplot as plt
import matplotlib.colors as colors

class SlopeCostmapPublisher(Node):
    def __init__(self):
        super().__init__('slope_costmap_publisher')
        self.costmap_pub = self.create_publisher(OccupancyGrid, '/slope_costmap', 10)
        self.pcd_file = '/home/kage/Downloads/Point_clouds/autonomous_ground_2025_2.pcd'
        self.resolution = 0.5
        
        # Clear slope thresholds
        self.gentle_slope = 15.0      # Green - Easy traversal
        self.moderate_slope = 25.0    # Yellow - Moderate difficulty  
        self.steep_slope = 40.0      # Orange - Difficult
        self.dangerous_slope = 50.0  # Red - Dangerous/Avoid
        
        self.timer = self.create_timer(0.5, self.publish_costmap)
        self.get_logger().info('Computing slope-based costmap...')
        self.create_slope_costmap()

    def create_slope_costmap(self):
        # Load point cloud
        pcd = o3d.io.read_point_cloud(self.pcd_file)
        pts = np.asarray(pcd.points)
        if pts.size == 0:
            self.get_logger().error('Empty point cloud')
            return

        self.get_logger().info(f'Loaded {len(pts)} points')

        # Grid bounds
        min_x, max_x = pts[:,0].min(), pts[:,0].max()
        min_y, max_y = pts[:,1].min(), pts[:,1].max()
        width = int((max_x - min_x) / self.resolution) + 1
        height = int((max_y - min_y) / self.resolution) + 1

        # Organize points into grid cells
        cell_points = [[[] for _ in range(width)] for _ in range(height)]
        for x, y, z in pts:
            j = int((x - min_x) / self.resolution)
            i = int((y - min_y) / self.resolution)
            if 0 <= i < height and 0 <= j < width:
                cell_points[i][j].append(z)

        # Compute elevation map
        elev_map = np.full((height, width), np.nan)
        for i in range(height):
            for j in range(width):
                if cell_points[i][j]:
                    elev_map[i,j] = np.mean(cell_points[i][j])

        # Fill empty cells with interpolation
        mask = ~np.isnan(elev_map)
        if np.sum(mask) > 0:
            from scipy.interpolate import griddata
            valid_coords = np.column_stack(np.where(mask))
            valid_values = elev_map[mask]
            
            all_coords = np.column_stack(np.where(np.isnan(elev_map)))
            if len(all_coords) > 0:
                interpolated = griddata(valid_coords, valid_values, all_coords, method='nearest')
                elev_map[np.isnan(elev_map)] = interpolated

        # Smooth elevation map
        elev_map = ndimage.gaussian_filter(elev_map, sigma=1.0)

        # Compute slopes
        dz_dy, dz_dx = np.gradient(elev_map, self.resolution)
        slope_rad = np.arctan(np.hypot(dz_dx, dz_dy))
        slope_deg = np.degrees(slope_rad)

        # CLEAR COST ASSIGNMENT
        cost_map = np.zeros_like(slope_deg, dtype=np.uint8)
        
        # 0-2°: SAFE - Very low cost (0-20)
        mask_safe = slope_deg <= self.gentle_slope
        cost_map[mask_safe] = (slope_deg[mask_safe] / self.gentle_slope * 20).astype(np.uint8)
        
        # 2-5°: CAUTION - Low to moderate cost (20-40) 
        mask_caution = (slope_deg > self.gentle_slope) & (slope_deg <= self.moderate_slope)
        cost_map[mask_caution] = (20 + (slope_deg[mask_caution] - self.gentle_slope) / 
                                 (self.moderate_slope - self.gentle_slope) * 20).astype(np.uint8)
        
        # 5-10°: DIFFICULT - Moderate to high cost (40-70)
        mask_difficult = (slope_deg > self.moderate_slope) & (slope_deg <= self.steep_slope)
        cost_map[mask_difficult] = (40 + (slope_deg[mask_difficult] - self.moderate_slope) / 
                                   (self.steep_slope - self.moderate_slope) * 30).astype(np.uint8)
        
        # 10-15°: DANGEROUS - High cost (70-90)
        mask_dangerous = (slope_deg > self.steep_slope) & (slope_deg <= self.dangerous_slope)
        cost_map[mask_dangerous] = (70 + (slope_deg[mask_dangerous] - self.steep_slope) / 
                                   (self.dangerous_slope - self.steep_slope) * 20).astype(np.uint8)
        
        # >15°: AVOID - Maximum cost (90-100)
        mask_avoid = slope_deg > self.dangerous_slope
        cost_map[mask_avoid] = 90 + np.minimum((slope_deg[mask_avoid] - self.dangerous_slope) / 10 * 10, 10).astype(np.uint8)

        # Log detailed statistics
        self.log_terrain_stats(slope_deg, cost_map)

        # Build OccupancyGrid
        og = OccupancyGrid()
        og.header.frame_id = 'map'
        og.info.resolution = self.resolution
        og.info.width = width
        og.info.height = height
        og.info.origin.position.x = float(min_x)
        og.info.origin.position.y = float(min_y)
        og.info.origin.orientation.w = 1.0
        og.data = cost_map.flatten().tolist()
        self.occupancy_grid = og

        # Save multiple visualizations
        self.save_visualizations(cost_map, slope_deg, elev_map, min_x, min_y)

    def log_terrain_stats(self, slope_deg, cost_map):
        """Log detailed terrain analysis"""
        total_cells = slope_deg.size
        
        safe = np.sum(slope_deg <= self.gentle_slope)
        caution = np.sum((slope_deg > self.gentle_slope) & (slope_deg <= self.moderate_slope))
        difficult = np.sum((slope_deg > self.moderate_slope) & (slope_deg <= self.steep_slope))
        dangerous = np.sum((slope_deg > self.steep_slope) & (slope_deg <= self.dangerous_slope))
        avoid = np.sum(slope_deg > self.dangerous_slope)
        
        self.get_logger().info("=== TERRAIN ANALYSIS ===")
        self.get_logger().info(f"SAFE (0-{self.gentle_slope}°): {safe} cells ({safe/total_cells*100:.1f}%)")
        self.get_logger().info(f"CAUTION ({self.gentle_slope}-{self.moderate_slope}°): {caution} cells ({caution/total_cells*100:.1f}%)")
        self.get_logger().info(f"DIFFICULT ({self.moderate_slope}-{self.steep_slope}°): {difficult} cells ({difficult/total_cells*100:.1f}%)")
        self.get_logger().info(f"DANGEROUS ({self.steep_slope}-{self.dangerous_slope}°): {dangerous} cells ({dangerous/total_cells*100:.1f}%)")
        self.get_logger().info(f"AVOID (>{self.dangerous_slope}°): {avoid} cells ({avoid/total_cells*100:.1f}%)")
        self.get_logger().info(f"Slope range: {slope_deg.min():.2f}° to {slope_deg.max():.2f}°")
        self.get_logger().info(f"Cost range: {cost_map.min()} to {cost_map.max()}")

    def save_visualizations(self, cost_map, slope_deg, elev_map, origin_x, origin_y):
        """Save multiple visualization formats"""
        
        # 1. Standard PGM for ROS (inverted: 0=black/occupied, 100=white/free)
        pgm_standard = 255 - (cost_map * 255 // 100)
        img_standard = Image.fromarray(np.flipud(pgm_standard.astype(np.uint8)), mode='L')
        img_standard.save('/home/kage/slope_costmap.pgm')
        
        # 2. Intuitive visualization (0=white/safe, 100=black/dangerous)
        pgm_intuitive = (cost_map * 255 // 100).astype(np.uint8)
        img_intuitive = Image.fromarray(np.flipud(pgm_intuitive), mode='L')
        img_intuitive.save('/home/kage/slope_costmap_intuitive.pgm')
        
        # 3. Color-coded visualization using matplotlib
        plt.figure(figsize=(12, 10))
        
        # Create custom colormap: Green -> Yellow -> Orange -> Red
        colors_list = ['green', 'yellow', 'orange', 'red', 'darkred']
        n_bins = 100
        cmap = colors.LinearSegmentedColormap.from_list('terrain', colors_list, N=n_bins)
        
        # Plot with clear boundaries
        plt.subplot(2, 2, 1)
        im1 = plt.imshow(cost_map, cmap=cmap, vmin=0, vmax=100, origin='lower')
        plt.colorbar(im1, label='Traversal Cost')
        plt.title('Traversal Cost Map\n(Green=Safe, Red=Dangerous)')
        
        # Add contour lines for slope categories
        plt.contour(cost_map, levels=[20, 40, 70, 90], colors='black', linewidths=0.5, alpha=0.7)
        
        # Plot slope map
        plt.subplot(2, 2, 2)
        im2 = plt.imshow(slope_deg, cmap='terrain', origin='lower')
        plt.colorbar(im2, label='Slope (degrees)')
        plt.title('Raw Slope Map')
        
        # Plot elevation map
        plt.subplot(2, 2, 3)
        im3 = plt.imshow(elev_map, cmap='terrain', origin='lower')
        plt.colorbar(im3, label='Elevation (m)')
        plt.title('Elevation Map')
        
        # Plot histogram of slopes
        plt.subplot(2, 2, 4)
        plt.hist(slope_deg.flatten(), bins=50, alpha=0.7, edgecolor='black')
        plt.axvline(self.gentle_slope, color='green', linestyle='--', label=f'Safe limit ({self.gentle_slope}°)')
        plt.axvline(self.moderate_slope, color='yellow', linestyle='--', label=f'Caution limit ({self.moderate_slope}°)')
        plt.axvline(self.steep_slope, color='orange', linestyle='--', label=f'Difficult limit ({self.steep_slope}°)')
        plt.axvline(self.dangerous_slope, color='red', linestyle='--', label=f'Danger limit ({self.dangerous_slope}°)')
        plt.xlabel('Slope (degrees)')
        plt.ylabel('Frequency')
        plt.title('Slope Distribution')
        plt.legend()
        plt.grid(True, alpha=0.3)
        
        plt.tight_layout()
        plt.savefig('/home/kage/slope_analysis_2.png', dpi=300, bbox_inches='tight')
        plt.close()
        
        # 4. Create legend image
        self.create_legend()
        
        # Save YAML
        yaml_content = {
            'image': 'slope_costmap.pgm',
            'resolution': float(self.resolution),
            'origin': [float(origin_x), float(origin_y), 0.0],
            'negate': 0,
            'occupied_thresh': 0.7,
            'free_thresh': 0.3
        }
        with open('/home/kage/slope_costmap.yaml', 'w') as f:
            yaml.dump(yaml_content, f, default_flow_style=False)
        
        self.get_logger().info("Saved visualizations:")
        self.get_logger().info("  - slope_costmap.pgm (ROS standard)")
        self.get_logger().info("  - slope_costmap_intuitive.pgm (intuitive)")
        self.get_logger().info("  - slope_analysis.png (detailed analysis)")
        self.get_logger().info("  - legend.png (interpretation guide)")

    def create_legend(self):
        """Create a legend explaining the costmap"""
        fig, ax = plt.subplots(figsize=(8, 6))
        
        # Create sample data for legend
        legend_data = np.array([
            [10, 25, 50, 80, 95],  # Cost values
        ])
        
        colors_list = ['green', 'yellow', 'orange', 'red', 'darkred']
        cmap = colors.LinearSegmentedColormap.from_list('terrain', colors_list, N=100)
        
        im = ax.imshow(legend_data, cmap=cmap, vmin=0, vmax=100, aspect='auto')
        
        # Add labels
        labels = ['SAFE\n(0-2°)', 'CAUTION\n(2-5°)', 'DIFFICULT\n(5-10°)', 'DANGEROUS\n(10-15°)', 'AVOID\n(>15°)']
        positions = [0, 1, 2, 3, 4]
        
        ax.set_xticks(positions)
        ax.set_xticklabels(labels)
        ax.set_yticks([])
        ax.set_title('Slope Costmap Legend\n(Darker = More Dangerous)', fontsize=14, fontweight='bold')
        
        # Add colorbar
        cbar = plt.colorbar(im, ax=ax, orientation='horizontal', pad=0.2)
        cbar.set_label('Traversal Cost (0=Easy, 100=Impossible)', fontsize=12)
        
        plt.tight_layout()
        plt.savefig('/home/kage/legend.png', dpi=300, bbox_inches='tight')
        plt.close()

    def publish_costmap(self):
        if hasattr(self, 'occupancy_grid'):
            self.occupancy_grid.header.stamp = self.get_clock().now().to_msg()
            self.costmap_pub.publish(self.occupancy_grid)

def main(args=None):
    rclpy.init(args=args)
    node = SlopeCostmapPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()