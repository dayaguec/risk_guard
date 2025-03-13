#include <ros2_ipcamera/setup_synchronizer.hpp>

namespace ros2_ipcamera
{
  SetupSynchronizerNode::SetupSynchronizerNode(const std::string &node_name,
    const rclcpp::NodeOptions &options)
  : Node(node_name, options)
  {
    this->declare_parameter<std::string>("camera_left_topic", "");
    this->declare_parameter<std::string>("camera_right_topic", "");
    this->declare_parameter<std::string>("lidar_topic", "");
    this->declare_parameter<std::string>("aggregated_perception_topic", "");

    auto qos = rclcpp::QoS(rclcpp::KeepLast(1), rmw_qos_profile_sensor_data);
    camera_left_sub_.subscribe(this, this->get_parameter("camera_left_topic").as_string(), qos.get_rmw_qos_profile());
    camera_right_sub_.subscribe(this, this->get_parameter("camera_right_topic").as_string(), qos.get_rmw_qos_profile());
    lidar_sub_.subscribe(this, this->get_parameter("lidar_topic").as_string(), qos.get_rmw_qos_profile());

    aggregated_perception_pub_ = this->create_publisher<ros2_ipcamera::msg::AggregatedPerception>(
      this->get_parameter("aggregated_perception_topic").as_string(), 1);

    using namespace std::placeholders;
    synchronizer_.reset(new message_synchronizer(sync_policy(10), camera_left_sub_, camera_right_sub_, lidar_sub_));
    synchronizer_->registerCallback(
      std::bind(&SetupSynchronizerNode::sync_callback, this, _1, _2, _3));
  }

  void SetupSynchronizerNode::sync_callback(
  	const sensor_msgs::msg::CompressedImage::ConstSharedPtr camera_left_msg,
  	const sensor_msgs::msg::CompressedImage::ConstSharedPtr camera_right_msg,
  	const sensor_msgs::msg::PointCloud2::ConstSharedPtr lidar_msg) const
  {
  	ros2_ipcamera::msg::AggregatedPerception aggregated_perception_msg;
  	aggregated_perception_msg.left_image = *camera_left_msg;
  	aggregated_perception_msg.right_image = *camera_right_msg;
  	aggregated_perception_msg.points = *lidar_msg;

  	aggregated_perception_pub_->publish(aggregated_perception_msg);

    RCLCPP_INFO_STREAM(get_logger(), "Synch Camera and Lidar data...");
    RCLCPP_INFO_STREAM(get_logger(), "Camera Left Stamp: " << camera_left_msg->header.stamp.sec);
    RCLCPP_INFO_STREAM(get_logger(), "Camera Right Stamp: " << camera_right_msg->header.stamp.sec);
    RCLCPP_INFO_STREAM(get_logger(), "LiDAR Stamp: " << lidar_msg->header.stamp.sec);
  }
}
