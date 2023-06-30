#ifndef IPCAMERA_NODE_HPP
#define IPCAMERA_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/header.hpp>
#include <chrono>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>
#include <opencv2/opencv.hpp>
#include <camera_info_manager/camera_info_manager.hpp>

namespace ros2_ipcamera
{
  class IpCameraNode : public rclcpp::Node
  {
    public:
      IpCameraNode(const std::string &node_name,
        const rclcpp::NodeOptions &options);

    private:
      void execute();
      std::string mat_type2encoding(const int mat_type);
      void convert_frame_to_message(const cv::Mat &frame,
        sensor_msgs::msg::Image &msg);

      sensor_msgs::msg::CameraInfo camera_info_;
      std::string frame_id_;
      rclcpp::TimerBase::SharedPtr timer_;

      image_transport::CameraPublisher pub_;

      cv::VideoCapture cap_;
  };
}  // namespace ros2_ipcamera
#endif // IPCAMERA_NODE_HPP
