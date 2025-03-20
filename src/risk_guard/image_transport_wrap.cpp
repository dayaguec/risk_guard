#include <risk_guard/image_transport_wrap.hpp>

namespace risk_guard
{
  ImageTransportNode::ImageTransportNode()
    : Node("image_transport_node")
  {
    this->declare_parameter<std::string>("sensor_params", "");
    this->declare_parameter<std::string>("in_transport", "raw");
  }

  bool ImageTransportNode::initialize()
  {
    try
    {
      std::string params = this->get_parameter("sensor_params").as_string();
      RCLCPP_INFO_STREAM(this->get_logger(), "Reading sensor params in " << params);
      YAML::Node config = YAML::LoadFile(params);
      if(config["cameras"])
      {
        typedef void (image_transport::Publisher::* PublishMemFn)(
          const sensor_msgs::msg::Image::ConstSharedPtr &) const;
        PublishMemFn pub_mem_fn = &image_transport::Publisher::publish;

        config = config["cameras"];
        for(unsigned int ii = 0; ii < config.size(); ++ii)
        {
          std::string out_topic = rclcpp::expand_topic_or_service_name(
            config[ii].as<std::string>() + "/image_it",
            this->get_name(), this->get_namespace());
          RCLCPP_DEBUG_STREAM(this->get_logger(), out_topic);

          std::string in_topic = rclcpp::expand_topic_or_service_name(
            config[ii].as<std::string>(),
            this->get_name(), this->get_namespace());
          RCLCPP_DEBUG_STREAM(this->get_logger(), in_topic);

          auto pub = image_transport::create_publisher(this, out_topic);
          it_publishers_.push_back(pub);

          auto sub = image_transport::create_subscription(
            this, in_topic, std::bind(pub_mem_fn, pub, std::placeholders::_1),
            this->get_parameter("in_transport").as_string());
          it_subscribers_.push_back(sub);
        }
      }
      else
      {
        RCLCPP_ERROR(this->get_logger(),
          "Camera info not found in config file, exiting...!");
        return false;
      }
    }
    catch(const std::exception &e)
    {
      RCLCPP_ERROR_STREAM(this->get_logger(),
        "Error reading config file, " << e.what());
      return false;
    }
    return true;
  }
}
