#include <ros2_ipcamera/message_sync.hpp>

namespace message_sync
{
  MessageSyncNode::MessageSyncNode(const std::string &node_name,
    const rclcpp::NodeOptions &options)
  : Node(node_name, options),
    sync_approximate_(2)
  {
    this->declare_parameter<std::string>("fisheye_topic", "");
    this->declare_parameter<std::string>("hick_topic", "");
    this->declare_parameter<std::string>("ouster_topic", "");

    fisheye_sub_.subscribe(this, this->get_parameter("fisheye_topic").as_string());
    hick_sub_.subscribe(this, this->get_parameter("hick_topic").as_string());
    outer_sub_.subscribe(this, this->get_parameter("ouster_topic").as_string());

    using namespace std::placeholders;
    sync_approximate_.registerCallback(
      std::bind(&MessageSyncNode::sync_callback, this, _1, _2, _3));
  }

  void MessageSyncNode::sync_callback(
    const sensor_msgs::msg::Image::ConstSharedPtr fisheye_msg,
    const sensor_msgs::msg::Image::ConstSharedPtr hick_msg,
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr outer_msg) const
  {
    RCLCPP_INFO_STREAM(get_logger(),"Fisheye stamp: "
      << fisheye_msg->header.stamp.sec
      << "Hick stamp: " << hick_msg->header.stamp.sec
      << "Ouster stamp: " << outer_msg->header.stamp.sec);
  }
}
