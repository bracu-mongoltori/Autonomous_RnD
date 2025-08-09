# Depth Smoothing Package

A ROS2 C++ package for smoothing depth images from RealSense D435i camera and converting them to laser scan data with improved density and reduced noise.

## Features

- **Advanced Depth Smoothing**: Uses bilateral filtering, Gaussian blur, and median filtering to reduce noise while preserving edges
- **Row Density Enhancement**: Increases the number of scan rows to reduce gaps between laser scan layers
- **Configurable Parameters**: Extensive parameter tuning for different environments and requirements
- **Point Cloud Generation**: Outputs smoothed 3D point clouds alongside processed depth images
- **RealSense Integration**: Seamless integration with RealSense D435i camera

## Package Structure

```
depth_smoothing/
├── CMakeLists.txt
├── package.xml
├── config/
│   ├── depth_smoothing_params.yaml      # Smoothing parameters
│   ├── depth_to_laserscan_params.yaml   # Laser scan conversion parameters
│   └── smoothed_depth_visualization.rviz # RViz configuration
├── include/depth_smoothing/
│   └── depth_smoothing_node.hpp         # Node header
├── launch/
│   ├── realsense_depth_smoothing.launch.py  # Main launch file
│   └── view_smoothed_depth.launch.py        # Visualization launch file
└── src/
    └── depth_smoothing_node.cpp         # Main node implementation
```

## Dependencies

- ROS2 (tested with Humble)
- OpenCV
- PCL (Point Cloud Library)
- cv_bridge
- image_transport
- realsense2_camera
- depthimage_to_laserscan

## Installation

1. Clone this package to your ROS2 workspace:
```bash
cd ~/ros2_ws/src
git clone <repository_url> depth_smoothing
```

2. Install dependencies:
```bash
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
```

3. Build the package:
```bash
colcon build --packages-select depth_smoothing
```

4. Source the workspace:
```bash
source ~/ros2_ws/install/setup.bash
```

## Usage

### Launch the complete system (RealSense + Smoothing + LaserScan):

```bash
ros2 launch depth_smoothing realsense_depth_smoothing.launch.py
```

Optional parameters:
- `enable_depth:=true/false` - Enable depth camera (default: true)
- `enable_color:=true/false` - Enable color camera (default: true)  
- `depth_width:=640` - Depth image width (default: 640)
- `depth_height:=480` - Depth image height (default: 480)
- `depth_fps:=30` - Depth image FPS (default: 30)

### Launch visualization only:

```bash
ros2 launch depth_smoothing view_smoothed_depth.launch.py
```

### Launch individual components:

```bash
# RealSense camera only
ros2 launch realsense2_camera rs_launch.py enable_depth:=true enable_color:=true

# Depth smoothing node only
ros2 run depth_smoothing depth_smoothing_node --ros-args --params-file src/depth_smoothing/config/depth_smoothing_params.yaml

# Depth to laserscan only  
ros2 run depthimage_to_laserscan depthimage_to_laserscan_node --ros-args --params-file src/depth_smoothing/config/depth_to_laserscan_params.yaml
```

## Topics

### Subscribed Topics:
- `/camera/camera/depth/image_rect_raw` (sensor_msgs/Image) - Input depth image
- `/camera/camera/depth/camera_info` (sensor_msgs/CameraInfo) - Camera calibration

### Published Topics:
- `/camera/depth/smoothed_image` (sensor_msgs/Image) - Smoothed depth image
- `/camera/depth/smoothed_points` (sensor_msgs/PointCloud2) - Smoothed point cloud
- `/scan` (sensor_msgs/LaserScan) - Generated laser scan

## Parameters

### Depth Smoothing Parameters (`config/depth_smoothing_params.yaml`):

- `gaussian_kernel_size`: Gaussian filter kernel size (default: 5)
- `gaussian_sigma`: Gaussian filter standard deviation (default: 1.0)
- `median_kernel_size`: Median filter kernel size (default: 3)
- `use_bilateral_filter`: Use bilateral filter for edge preservation (default: true)
- `bilateral_sigma_color`: Bilateral filter color sigma (default: 50.0)
- `bilateral_sigma_space`: Bilateral filter space sigma (default: 50.0)
- `row_interpolation_factor`: Factor to increase row density (default: 3)
- `depth_scale`: Depth value to meter conversion (default: 0.001)
- `max_depth`: Maximum depth in meters (default: 10.0)
- `min_depth`: Minimum depth in meters (default: 0.1)

### LaserScan Parameters (`config/depth_to_laserscan_params.yaml`):

- `output_frame_id`: Output frame for laser scan (default: "camera_depth_frame")
- `range_min/max`: Range limits in meters
- `scan_height`: Number of pixel rows for scan generation
- `angle_min/max`: Angular limits in radians
- `angle_increment`: Angular resolution

## Tuning Guide

### For Different Environments:

**Indoor/Close Range:**
- Decrease `max_depth` to 5.0
- Increase `bilateral_sigma_color` to 80.0
- Use `row_interpolation_factor` of 2-3

**Outdoor/Long Range:**
- Increase `max_depth` to 15.0
- Decrease `bilateral_sigma_color` to 30.0
- Use `row_interpolation_factor` of 3-4

**High Noise Environment:**
- Increase `median_kernel_size` to 5
- Increase `gaussian_sigma` to 1.5
- Enable bilateral filter

**Real-time Performance:**
- Disable bilateral filter
- Reduce `row_interpolation_factor` to 2
- Reduce `gaussian_kernel_size` to 3

## Troubleshooting

1. **No depth data**: Check RealSense camera connection and launch parameters
2. **High CPU usage**: Reduce image resolution or disable bilateral filter
3. **Noisy laser scan**: Increase smoothing parameters or median filter size
4. **Missing scan data**: Check depth range parameters and camera calibration

## License

MIT License - see LICENSE file for details.
