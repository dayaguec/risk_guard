#ifndef LIDAR_CAMERA_WRITTER_NODE_HPP
#define LIDAR_CAMERA_WRITTER_NODE_HPP

#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>

#include <pcl/common/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>

namespace risk_guard
{
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::CompressedImage,
    sensor_msgs::msg::PointCloud2> sync_policy;
  typedef message_filters::Synchronizer<sync_policy> message_synchronizer;

  class LidarCameraWriterNode : public rclcpp::Node
  {
    public:
      LidarCameraWriterNode(const std::string &node_name,
        const rclcpp::NodeOptions &options);

    private:
      message_filters::Subscriber<sensor_msgs::msg::CompressedImage> camera_sub_;
      message_filters::Subscriber<sensor_msgs::msg::PointCloud2> lidar_sub_;
      
      std::shared_ptr<message_synchronizer> synchronizer_;

      std::string camera_save_path_;
      std::string lidar_save_path_;

      void sync_callback(
        const sensor_msgs::msg::CompressedImage::ConstSharedPtr camera_msg,
        const sensor_msgs::msg::PointCloud2::ConstSharedPtr lidar_msg) const;
  };
}  // namespace risk_guard
#endif // LIDAR_CAMERA_WRITTER_NODE_HPP
