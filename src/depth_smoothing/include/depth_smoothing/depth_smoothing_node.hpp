#ifndef DEPTH_SMOOTHING__DEPTH_SMOOTHING_NODE_HPP_
#define DEPTH_SMOOTHING__DEPTH_SMOOTHING_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <memory>

namespace depth_smoothing
{

class DepthSmoothingNode : public rclcpp::Node
{
public:
  explicit DepthSmoothingNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  // Callback functions
  void depth_callback(const sensor_msgs::msg::Image::SharedPtr msg);
  void camera_info_callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg);
  
  // Processing functions
  cv::Mat smooth_depth_image(const cv::Mat & depth_image);
  cv::Mat increase_row_density(const cv::Mat & depth_image);
  sensor_msgs::msg::PointCloud2 depth_to_pointcloud(const cv::Mat & depth_image, 
                                                     const std_msgs::msg::Header & header);
  
  // Subscribers
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
  
  // Publishers
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr smoothed_depth_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_pub_;
  
  // Camera parameters
  sensor_msgs::msg::CameraInfo::SharedPtr camera_info_;
  bool camera_info_received_;
  
  // Parameters
  int gaussian_kernel_size_;
  double gaussian_sigma_;
  int median_kernel_size_;
  bool use_bilateral_filter_;
  double bilateral_sigma_color_;
  double bilateral_sigma_space_;
  int row_interpolation_factor_;
  double depth_scale_;
  double max_depth_;
  double min_depth_;
  
  // Frame names
  std::string camera_frame_;
  std::string output_frame_;
  
  // TF
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

} // namespace depth_smoothing

#endif // DEPTH_SMOOTHING__DEPTH_SMOOTHING_NODE_HPP_
