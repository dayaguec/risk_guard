#include <risk_guard/image_transport_wrap.hpp>

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto it_node = std::make_shared<risk_guard::ImageTransportNode>();
  if(it_node->initialize())
  {
    rclcpp::spin(it_node);
  }
  rclcpp::shutdown();
  return 0;
}