#include "depth_smoothing/depth_smoothing_node.hpp"
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <pcl/filters/passthrough.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace depth_smoothing
{

DepthSmoothingNode::DepthSmoothingNode(const rclcpp::NodeOptions & options)
: Node("depth_smoothing_node", options), camera_info_received_(false)
{
  // Declare parameters
  this->declare_parameter("gaussian_kernel_size", 5);
  this->declare_parameter("gaussian_sigma", 1.0);
  this->declare_parameter("median_kernel_size", 5);
  this->declare_parameter("use_bilateral_filter", true);
  this->declare_parameter("bilateral_sigma_color", 50.0);
  this->declare_parameter("bilateral_sigma_space", 50.0);
  this->declare_parameter("row_interpolation_factor", 3);
  this->declare_parameter("depth_scale", 0.001);
  this->declare_parameter("max_depth", 10.0);
  this->declare_parameter("min_depth", 0.1);
  this->declare_parameter("camera_frame", "camera_depth_optical_frame");
  this->declare_parameter("output_frame", "camera_depth_optical_frame");
  
  // Get parameters
  gaussian_kernel_size_ = this->get_parameter("gaussian_kernel_size").as_int();
  gaussian_sigma_ = this->get_parameter("gaussian_sigma").as_double();
  median_kernel_size_ = this->get_parameter("median_kernel_size").as_int();
  use_bilateral_filter_ = this->get_parameter("use_bilateral_filter").as_bool();
  bilateral_sigma_color_ = this->get_parameter("bilateral_sigma_color").as_double();
  bilateral_sigma_space_ = this->get_parameter("bilateral_sigma_space").as_double();
  row_interpolation_factor_ = this->get_parameter("row_interpolation_factor").as_int();
  depth_scale_ = this->get_parameter("depth_scale").as_double();
  max_depth_ = this->get_parameter("max_depth").as_double();
  min_depth_ = this->get_parameter("min_depth").as_double();
  camera_frame_ = this->get_parameter("camera_frame").as_string();
  output_frame_ = this->get_parameter("output_frame").as_string();
  
  // Initialize TF
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  
  // Create subscribers
  depth_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
    "/camera/depth/image_rect_raw", 10,
    std::bind(&DepthSmoothingNode::depth_callback, this, std::placeholders::_1));
    
  camera_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
    "/camera/depth/camera_info", 10,
    std::bind(&DepthSmoothingNode::camera_info_callback, this, std::placeholders::_1));
  
  // Create publishers
  smoothed_depth_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
    "/camera/depth/smoothed_image", 10);
    
  pointcloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
    "/camera/depth/smoothed_points", 10);
  
  RCLCPP_INFO(this->get_logger(), "Depth smoothing node initialized");
  RCLCPP_INFO(this->get_logger(), "Gaussian kernel size: %d, sigma: %.2f", 
              gaussian_kernel_size_, gaussian_sigma_);
  RCLCPP_INFO(this->get_logger(), "Row interpolation factor: %d", row_interpolation_factor_);
  RCLCPP_INFO(this->get_logger(), "Using bilateral filter: %s", 
              use_bilateral_filter_ ? "true" : "false");
}

void DepthSmoothingNode::camera_info_callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
{
  camera_info_ = msg;
  camera_info_received_ = true;
  RCLCPP_INFO_ONCE(this->get_logger(), "Camera info received");
}

void DepthSmoothingNode::depth_callback(const sensor_msgs::msg::Image::SharedPtr msg)
{
  if (!camera_info_received_) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                          "Camera info not received yet, skipping depth processing");
    return;
  }
  
  try {
    // Convert ROS image to OpenCV
    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_16UC1);
    cv::Mat depth_image = cv_ptr->image;
    
    // Apply smoothing
    cv::Mat smoothed_depth = smooth_depth_image(depth_image);
    
    // Increase row density
    cv::Mat dense_depth = increase_row_density(smoothed_depth);
    
    // Publish smoothed depth image
    sensor_msgs::msg::Image::SharedPtr smoothed_msg = cv_bridge::CvImage(
      msg->header, sensor_msgs::image_encodings::TYPE_16UC1, dense_depth).toImageMsg();
    smoothed_depth_pub_->publish(*smoothed_msg);
    
    // Convert to point cloud and publish
    sensor_msgs::msg::PointCloud2 pointcloud_msg = depth_to_pointcloud(dense_depth, msg->header);
    pointcloud_pub_->publish(pointcloud_msg);
    
  } catch (cv_bridge::Exception& e) {
    RCLCPP_ERROR(this->get_logger(), "CV bridge exception: %s", e.what());
  }
}

cv::Mat DepthSmoothingNode::smooth_depth_image(const cv::Mat & depth_image)
{
  cv::Mat smoothed;
  cv::Mat float_depth;
  
  // Convert to float for better processing
  depth_image.convertTo(float_depth, CV_32F);
  
  // Create mask for valid pixels (non-zero depth values)
  cv::Mat mask = (float_depth > 0) & (float_depth < 65535);
  
  if (use_bilateral_filter_) {
    // Apply bilateral filter for edge-preserving smoothing
    cv::bilateralFilter(float_depth, smoothed, -1, bilateral_sigma_color_, bilateral_sigma_space_);
  } else {
    // Apply Gaussian blur
    cv::GaussianBlur(float_depth, smoothed, 
                     cv::Size(gaussian_kernel_size_, gaussian_kernel_size_), 
                     gaussian_sigma_);
  }
  
  // Apply median filter for noise reduction
  if (median_kernel_size_ > 1) {
    cv::Mat temp;
    smoothed.convertTo(temp, CV_16UC1);
    cv::medianBlur(temp, temp, median_kernel_size_);
    temp.convertTo(smoothed, CV_32F);
  }
  
  // Apply mask to keep only valid pixels
  smoothed.setTo(0, ~mask);
  
  // Convert back to 16-bit
  cv::Mat result;
  smoothed.convertTo(result, CV_16UC1);
  
  return result;
}

cv::Mat DepthSmoothingNode::increase_row_density(const cv::Mat & depth_image)
{
  cv::Mat dense_image;
  
  // Resize image to increase row density
  cv::resize(depth_image, dense_image, 
             cv::Size(depth_image.cols, depth_image.rows * row_interpolation_factor_),
             0, 0, cv::INTER_LINEAR);
  
  // Apply additional smoothing after interpolation
  cv::Mat float_dense;
  dense_image.convertTo(float_dense, CV_32F);
  
  // Light Gaussian blur to smooth interpolated values
  cv::GaussianBlur(float_dense, float_dense, cv::Size(3, 3), 0.5);
  
  cv::Mat result;
  float_dense.convertTo(result, CV_16UC1);
  
  return result;
}

sensor_msgs::msg::PointCloud2 DepthSmoothingNode::depth_to_pointcloud(
  const cv::Mat & depth_image, const std_msgs::msg::Header & header)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  
  // Camera intrinsics
  double fx = camera_info_->k[0];
  double fy = camera_info_->k[4];
  double cx = camera_info_->k[2];
  double cy = camera_info_->k[5];
  
  cloud->width = depth_image.cols;
  cloud->height = depth_image.rows;
  cloud->is_dense = false;
  cloud->points.resize(cloud->width * cloud->height);
  
  int point_index = 0;
  for (int v = 0; v < depth_image.rows; ++v) {
    for (int u = 0; u < depth_image.cols; ++u) {
      pcl::PointXYZ & point = cloud->points[point_index++];
      
      uint16_t depth_value = depth_image.at<uint16_t>(v, u);
      
      if (depth_value == 0) {
        point.x = point.y = point.z = std::numeric_limits<float>::quiet_NaN();
        continue;
      }
      
      // Convert depth to meters
      double depth = static_cast<double>(depth_value) * depth_scale_;
      
      // Filter by depth range
      if (depth < min_depth_ || depth > max_depth_) {
        point.x = point.y = point.z = std::numeric_limits<float>::quiet_NaN();
        continue;
      }
      
      // Project to 3D
      point.z = depth;
      point.x = (u - cx) * depth / fx;
      point.y = (v - cy) * depth / fy;
    }
  }
  
  // Convert to ROS message
  sensor_msgs::msg::PointCloud2 output;
  pcl::toROSMsg(*cloud, output);
  output.header = header;
  output.header.frame_id = output_frame_;
  
  return output;
}

} // namespace depth_smoothing

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<depth_smoothing::DepthSmoothingNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
