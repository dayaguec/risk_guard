#include <risk_guard/lidar_camera_writter.hpp>

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  auto node = std::make_shared<risk_guard::LidarCameraWriterNode>(
    "lidar_camera_writter_node", options);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
