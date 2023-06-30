#include <ros2_ipcamera/ros2_ipcamera.hpp>

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  auto node = std::make_shared<ros2_ipcamera::IpCameraNode>(
    "ip_camera_node", options);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
