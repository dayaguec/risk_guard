#ifndef IPMESSAGE_SYNCODE_HPP
#define IPMESSAGE_SYNCODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

namespace message_sync
{
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, sensor_msgs::msg::Image, sensor_msgs::msg::PointCloud2> approximate_policy;

  class MessageSyncNode : public rclcpp::Node
  {
    public:
      MessageSyncNode(const std::string &node_name,
        const rclcpp::NodeOptions &options);

    private:
      message_filters::Subscriber<sensor_msgs::msg::Image> fisheye_sub_;
      message_filters::Subscriber<sensor_msgs::msg::Image> hick_sub_;
      message_filters::Subscriber<sensor_msgs::msg::PointCloud2> outer_sub_;

      message_filters::Synchronizer<approximate_policy> sync_approximate_;

      void sync_callback(
        const sensor_msgs::msg::Image::ConstSharedPtr fisheye_msg,
        const sensor_msgs::msg::Image::ConstSharedPtr hick_msg,
        const sensor_msgs::msg::PointCloud2::ConstSharedPtr outer_msg) const;
  };
}  // namespace message_sync
#endif // MESSAGE_SYNC_NODE_HPP
