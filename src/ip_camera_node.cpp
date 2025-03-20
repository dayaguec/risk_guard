#include <risk_guard/ip_camera.hpp>

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  auto node = std::make_shared<risk_guard::IpCameraNode>(
    "ip_camera_node", options);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
