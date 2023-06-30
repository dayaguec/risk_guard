#include <ros2_ipcamera/message_sync.hpp>

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  auto node = std::make_shared<message_sync::MessageSyncNode>(
    "message_sync_node", options);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
