#include <ros2_ipcamera/ros2_ipcamera.hpp>

namespace ros2_ipcamera
{
  IpCameraNode::IpCameraNode(const std::string &node_name,
    const rclcpp::NodeOptions &options)
  : Node(node_name, options)
  {
    this->declare_parameter<std::string>("rtsp_uri", "");
    this->declare_parameter<std::string>("camera_calibration_file", "");
    this->declare_parameter<int>("image_width", 640);
    this->declare_parameter<int>("image_height", 480);
    this->declare_parameter<double>("node_rate", 10);
    this->declare_parameter<std::string>("image_topic", "image_raw");
    this->declare_parameter<std::string>("frame_id", "ip_camera");

    std::string source = this->get_parameter("rtsp_uri").as_string();
    int width = this->get_parameter("image_width").as_int();
    int height = this->get_parameter("image_height").as_int();
    double node_rate = this->get_parameter("node_rate").as_double();
    std::string camera_calibration_file_param = this->get_parameter("camera_calibration_file").as_string();
    std::string image_topic = this->get_parameter("image_topic").as_string();
    frame_id_ = this->get_parameter("frame_id").as_string();

    rclcpp::Logger node_logger = this->get_logger();
    RCLCPP_INFO(node_logger, "rtsp_uri: %s", source.c_str());
    RCLCPP_INFO(node_logger, "camera_calibration_file: %s", camera_calibration_file_param.c_str());
    RCLCPP_INFO(node_logger, "image_width: %d", width);
    RCLCPP_INFO(node_logger, "image_height: %d", height);
    RCLCPP_INFO(node_logger, "node_rate: %f", node_rate);
    RCLCPP_INFO(node_logger, "image_topic: %s", image_topic.c_str());
    RCLCPP_INFO(node_logger, "frame_id: %s", frame_id_.c_str());

    this->cap_.open(source);
    // Set the width and height based on command line arguments.
    // The width, height has to match the available resolutions of the IP camera.
    this->cap_.set(cv::CAP_PROP_FRAME_WIDTH, static_cast<double>(width));
    this->cap_.set(cv::CAP_PROP_FRAME_HEIGHT, static_cast<double>(height));
    if(!this->cap_.isOpened())
    {
      RCLCPP_ERROR(node_logger, "Could not open video stream");
      throw std::runtime_error("Could not open video stream");
    }

    std::shared_ptr<camera_info_manager::CameraInfoManager> cinfo_manager =
      std::make_shared<camera_info_manager::CameraInfoManager>(this);
    if(cinfo_manager->validateURL(camera_calibration_file_param))
    {
      cinfo_manager->loadCameraInfo(camera_calibration_file_param);
    }
    else
    {
      RCLCPP_WARN(node_logger, "CameraInfo URL not valid.");
      RCLCPP_WARN(node_logger, "URL IS %s", camera_calibration_file_param.c_str());
    }

    camera_info_ = cinfo_manager->getCameraInfo();

    auto qos = rclcpp::QoS(rclcpp::KeepLast(1), rmw_qos_profile_sensor_data);
    this->pub_ = image_transport::create_camera_publisher(
      this, image_topic, qos.get_rmw_qos_profile());

    timer_ = this->create_wall_timer(
      std::chrono::duration<double>(1.0/node_rate),
      std::bind(&IpCameraNode::execute, this));
  }

  void IpCameraNode::execute()
  {
    // Initialize OpenCV image matrices.
    cv::Mat frame;
    sensor_msgs::msg::Image image_msg;
    image_msg.is_bigendian = false;
    this->cap_ >> frame;

    if(!frame.empty())
    {
      // Convert to a ROS image
      convert_frame_to_message(frame, image_msg);
      // Publish the image message and increment the frame_id.
      this->pub_.publish(image_msg, camera_info_);
    }
    else
    {
      RCLCPP_WARN(this->get_logger(), "Video frame is empty dropping frame...");
    }
  }

  std::string IpCameraNode::mat_type2encoding(const int mat_type)
  {
    switch(mat_type)
    {
      case CV_8UC1:
        return "mono8";
      case CV_8UC3:
        return "bgr8";
      case CV_16SC1:
        return "mono16";
      case CV_8UC4:
        return "rgba8";
      default:
        throw std::runtime_error("Unsupported encoding type");
    }
  }

  void IpCameraNode::convert_frame_to_message(
    const cv::Mat &frame,
    sensor_msgs::msg::Image &msg)
  {
    // copy cv information into ros message
    msg.height = frame.rows;
    msg.width = frame.cols;
    msg.encoding = mat_type2encoding(frame.type());
    msg.step = static_cast<sensor_msgs::msg::Image::_step_type>(frame.step);
    size_t size = frame.step * frame.rows;
    msg.data.resize(size);
    memcpy(&msg.data[0], frame.data, size);

    rclcpp::Time timestamp = this->get_clock()->now();

    msg.header.frame_id = frame_id_;
    msg.header.stamp = timestamp;
    camera_info_.header.frame_id = frame_id_;
    camera_info_.header.stamp = timestamp;
  }
}
