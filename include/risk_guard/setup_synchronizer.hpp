#ifndef SETUP_SYNCHRONIZER_HPP
#define SETUP_SYNCHRONIZER_HPP

#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <risk_guard/msg/aggregated_perception.hpp>
#include <risk_guard/msg/aggregated_perception_compressed.hpp>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

#include <pcl/common/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>

namespace risk_guard
{
  #ifdef IMAGE_AND_PCL
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image,
  sensor_msgs::msg::Image, sensor_msgs::msg::PointCloud2> sync_policy;
  typedef message_filters::Synchronizer<sync_policy> message_synchronizer;
  #elif IMAGE_COMPRESS_AND_PCL
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::CompressedImage,
  sensor_msgs::msg::CompressedImage, sensor_msgs::msg::PointCloud2> sync_policy;
  typedef message_filters::Synchronizer<sync_policy> message_synchronizer;
  #elif ONLY_IMAGE
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image,
  sensor_msgs::msg::Image> sync_policy;
  typedef message_filters::Synchronizer<sync_policy> message_synchronizer;
  #else
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::CompressedImage,
  sensor_msgs::msg::CompressedImage> sync_policy;
  typedef message_filters::Synchronizer<sync_policy> message_synchronizer;
  #endif

  class SetupSynchronizerNode : public rclcpp::Node
  {
    public:
      SetupSynchronizerNode(const std::string &node_name,
        const rclcpp::NodeOptions &options);

    private:
      #ifdef IMAGE_AND_PCL
      message_filters::Subscriber<sensor_msgs::msg::Image> camera_right_sub_;
      message_filters::Subscriber<sensor_msgs::msg::Image> camera_left_sub_;
      message_filters::Subscriber<sensor_msgs::msg::PointCloud2> lidar_sub_;
      rclcpp::Publisher<risk_guard::msg::AggregatedPerception>::SharedPtr aggregated_perception_pub_;
      #elif IMAGE_COMPRESS_AND_PCL
      message_filters::Subscriber<sensor_msgs::msg::CompressedImage> camera_right_sub_;
      message_filters::Subscriber<sensor_msgs::msg::CompressedImage> camera_left_sub_;
      message_filters::Subscriber<sensor_msgs::msg::PointCloud2> lidar_sub_;
      rclcpp::Publisher<risk_guard::msg::AggregatedPerceptionCompressed>::SharedPtr aggregated_perception_pub_;
      #elif ONLY_IMAGE
      message_filters::Subscriber<sensor_msgs::msg::Image> camera_right_sub_;
      message_filters::Subscriber<sensor_msgs::msg::Image> camera_left_sub_;
      rclcpp::Publisher<risk_guard::msg::AggregatedPerception>::SharedPtr aggregated_perception_pub_;
      #else
      message_filters::Subscriber<sensor_msgs::msg::CompressedImage> camera_right_sub_;
      message_filters::Subscriber<sensor_msgs::msg::CompressedImage> camera_left_sub_;
      rclcpp::Publisher<risk_guard::msg::AggregatedPerceptionCompressed>::SharedPtr aggregated_perception_pub_;
      #endif

      std::shared_ptr<message_synchronizer> synchronizer_;

      #ifdef IMAGE_AND_PCL
      void sync_callback(
        const sensor_msgs::msg::Image::ConstSharedPtr camera_left_msg,
        const sensor_msgs::msg::Image::ConstSharedPtr camera_right_msg,
        const sensor_msgs::msg::PointCloud2::ConstSharedPtr lidar_msg) const;
      #elif IMAGE_COMPRESS_AND_PCL
      void sync_callback(
        const sensor_msgs::msg::CompressedImage::ConstSharedPtr camera_left_msg,
        const sensor_msgs::msg::CompressedImage::ConstSharedPtr camera_right_msg,
        const sensor_msgs::msg::PointCloud2::ConstSharedPtr lidar_msg) const;
      #elif ONLY_IMAGE
      void sync_callback(
        const sensor_msgs::msg::Image::ConstSharedPtr camera_left_msg,
        const sensor_msgs::msg::Image::ConstSharedPtr camera_right_msg) const;
      #else
      void sync_callback(
        const sensor_msgs::msg::CompressedImage::ConstSharedPtr camera_left_msg,
        const sensor_msgs::msg::CompressedImage::ConstSharedPtr camera_right_msg) const;
      #endif
  };
}  // namespace risk_guard
#endif // SETUP_SYNCHRONIZER_HPP
