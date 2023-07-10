#include <ros2_ipcamera/lidar_camera_writter.hpp>

namespace ros2_ipcamera
{
  LidarCameraWriterNode::LidarCameraWriterNode(const std::string &node_name,
    const rclcpp::NodeOptions &options)
  : Node(node_name, options)
  {
    this->declare_parameter<std::string>("camera_topic", "");
    this->declare_parameter<std::string>("lidar_topic", "");
    this->declare_parameter<std::string>("camera_folder", "");
    this->declare_parameter<std::string>("lidar_folder", "");

    camera_save_path_ = this->get_parameter("camera_folder").as_string();
    lidar_save_path_ = this->get_parameter("lidar_folder").as_string();

    auto qos = rclcpp::QoS(rclcpp::KeepLast(1), rmw_qos_profile_sensor_data);
    camera_sub_.subscribe(this, this->get_parameter("camera_topic").as_string(), qos.get_rmw_qos_profile());
    lidar_sub_.subscribe(this, this->get_parameter("lidar_topic").as_string(), qos.get_rmw_qos_profile());

    using namespace std::placeholders;
    synchronizer_.reset(new message_synchronizer(sync_policy(10), camera_sub_, lidar_sub_));
    synchronizer_->registerCallback(
      std::bind(&LidarCameraWriterNode::sync_callback, this, _1, _2));
  }

  void LidarCameraWriterNode::sync_callback(
    const sensor_msgs::msg::CompressedImage::ConstSharedPtr camera_msg,
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr lidar_msg) const
  {
    std::string fullpath = camera_save_path_ + std::to_string(camera_msg->header.stamp.sec)
      + std::string("_") + std::to_string(camera_msg->header.stamp.nanosec) + std::string(".jpg");
    cv_bridge::CvImagePtr cv_ptr;
    cv_ptr = cv_bridge::toCvCopy(camera_msg, "");
    cv::imwrite(fullpath, cv_ptr->image);

    fullpath = lidar_save_path_ + std::to_string(lidar_msg->header.stamp.sec)
      + std::string("_") + std::to_string(lidar_msg->header.stamp.nanosec) + std::string(".pcd");
    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(*lidar_msg, pcl_pc2);
    pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromPCLPointCloud2(pcl_pc2, *temp_cloud);
    pcl::io::savePCDFile(fullpath, *temp_cloud, true);

    RCLCPP_INFO_STREAM(get_logger(), "Writting Camera and Lidar data...");
  }
}
