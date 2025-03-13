#include <ros2_ipcamera/setup_synchronizer.hpp>

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  auto node = std::make_shared<ros2_ipcamera::SetupSynchronizerNode>(
    "setup_synchronizer_node", options);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
