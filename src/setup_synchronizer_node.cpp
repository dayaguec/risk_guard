#include <risk_guard/setup_synchronizer.hpp>

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  auto node = std::make_shared<risk_guard::SetupSynchronizerNode>(
    "setup_synchronizer_node", options);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
