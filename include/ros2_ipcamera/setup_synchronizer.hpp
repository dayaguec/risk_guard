#ifndef SETUP_SYNCHRONIZER_HPP
#define SETUP_SYNCHRONIZER_HPP

#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <ros2_ipcamera/msg/aggregated_perception.hpp>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

#include <pcl/common/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>

namespace ros2_ipcamera
{
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::CompressedImage,
  sensor_msgs::msg::CompressedImage, sensor_msgs::msg::PointCloud2> sync_policy;
  typedef message_filters::Synchronizer<sync_policy> message_synchronizer;

  class SetupSynchronizerNode : public rclcpp::Node
  {
    public:
      SetupSynchronizerNode(const std::string &node_name,
        const rclcpp::NodeOptions &options);

    private:
      message_filters::Subscriber<sensor_msgs::msg::CompressedImage> camera_right_sub_;
      message_filters::Subscriber<sensor_msgs::msg::CompressedImage> camera_left_sub_;
      message_filters::Subscriber<sensor_msgs::msg::PointCloud2> lidar_sub_;
      
      rclcpp::Publisher<ros2_ipcamera::msg::AggregatedPerception>::SharedPtr aggregated_perception_pub_;

      std::shared_ptr<message_synchronizer> synchronizer_;

      void sync_callback(
        const sensor_msgs::msg::CompressedImage::ConstSharedPtr camera_left_msg,
        const sensor_msgs::msg::CompressedImage::ConstSharedPtr camera_right_msg,
        const sensor_msgs::msg::PointCloud2::ConstSharedPtr lidar_msg) const;
  };
}  // namespace ros2_ipcamera
#endif // SETUP_SYNCHRONIZER_HPP
