#include <image_transport/image_transport.hpp>
#include <rclcpp/rclcpp.hpp>

#include <image_transport/image_transport.hpp>

#include <camera_info_manager/camera_info_manager.hpp>
#include <image_geometry/pinhole_camera_model.h>

// Include files to use OpenCV API.
#include <opencv2/core/mat.hpp>
#include <opencv2/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

// Include files to use the PYLON API.
#include <pylon/PylonIncludes.h>

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  auto basler_node = std::make_shared<rclcpp::Node>("basler_ros_node", options);
  basler_node->declare_parameter<int>("serial_number", 0);
  basler_node->declare_parameter<std::string>("device_user_id", "basler");
  basler_node->declare_parameter<double>("node_rate", 30);
  basler_node->declare_parameter<std::string>("camera_info_url", "");
    
  const std::string camera_info_url = basler_node->get_parameter("camera_info_url").as_string();

  rclcpp::WallRate loop_rate(
    basler_node->get_parameter("node_rate").as_double());

  image_transport::ImageTransport it(basler_node);
  const std::string device_user_id = basler_node->get_parameter("device_user_id").as_string();
  image_transport::Publisher rect_pub;
  image_transport::CameraPublisher raw_pub = image_transport::create_camera_publisher(
    basler_node.get(), device_user_id + "/image_raw");

  std::shared_ptr<image_geometry::PinholeCameraModel> pinhole_model;

  int exit_code = 0;

  std::shared_ptr<camera_info_manager::CameraInfoManager> camera_info_manager = std::make_shared<camera_info_manager::CameraInfoManager>(basler_node.get());
  if(!camera_info_manager->setCameraName(device_user_id))
  { 
    RCLCPP_WARN_STREAM(basler_node->get_logger(),
      "[" << device_user_id << "] name not valid for camera_info_manager!");
    rclcpp::shutdown();
    return exit_code;
  }
  sensor_msgs::msg::CameraInfo init_cam_info;
  init_cam_info.header.frame_id = device_user_id;
  init_cam_info.header.stamp = basler_node->now();

  init_cam_info.height = 1920;
  init_cam_info.width = 1080;
  init_cam_info.distortion_model = "";
  init_cam_info.d = std::vector<double>(5, 0.);
  init_cam_info.k.fill(0.0);
  init_cam_info.r.fill(0.0);
  init_cam_info.p.fill(0.0);
  init_cam_info.binning_x = 0;
  init_cam_info.binning_y = 0;
  init_cam_info.roi.x_offset = init_cam_info.roi.y_offset = 0;
  init_cam_info.roi.height = init_cam_info.roi.width = 0;

  camera_info_manager->setCameraInfo(init_cam_info);

  if(camera_info_url.empty() || !camera_info_manager->validateURL(camera_info_url))
  {
    RCLCPP_WARN(basler_node->get_logger(),
      "Camera Info URL is needed to provide rectify images! Node will only provide raw images...");
  }
  else
  {
    if(camera_info_manager->loadCameraInfo(camera_info_url))
    {
      rect_pub = it.advertise(device_user_id + "/image_rect", 1);
      sensor_msgs::msg::CameraInfo new_cam_info = camera_info_manager->getCameraInfo();
      new_cam_info.header.frame_id = device_user_id;
      new_cam_info.header.stamp = basler_node->now();
      camera_info_manager->setCameraInfo(new_cam_info);
      
      pinhole_model = std::make_shared<image_geometry::PinholeCameraModel>();
      pinhole_model->fromCameraInfo(camera_info_manager->getCameraInfo());
    }
    else
    { 
      RCLCPP_WARN(basler_node->get_logger(),
        "Unable to set Camera Info properly! Node will only provide distorted raw images!");
    }
  }

  Pylon::PylonAutoInitTerm auto_init_term;

  try
  {
    /// Todo: if no serial is provided get the first camera available
    const std::string serial_n = std::to_string(
      basler_node->get_parameter("serial_number").as_int());
    RCLCPP_INFO_STREAM(basler_node->get_logger(),
      "Creating Camera with serial number: " << serial_n);
    Pylon::CDeviceInfo camera_info;
    camera_info.SetSerialNumber(serial_n.c_str());
    Pylon::CInstantCamera camera(Pylon::CTlFactory::GetInstance().CreateFirstDevice(camera_info));
  
    RCLCPP_INFO_STREAM(basler_node->get_logger(), "Camera Created..." <<
      std::endl << "Using device: " << camera.GetDeviceInfo().GetModelName());

    /* The parameter MaxNumBuffer can be used to control the count of buffers
       allocated for grabbing. The default value of this parameter is 10. */
    camera.MaxNumBuffer = 20;

    // Create pylon image format converter and pylon image
    Pylon::CImageFormatConverter format_converter;
    format_converter.OutputPixelFormat = Pylon::PixelType_BGR8packed;
    Pylon::CPylonImage pylon_image;

    // Create an OpenCV image
    cv::Mat open_cv_image, open_cv_image_rect, resized_open_cv_image;
    std_msgs::msg::Header hdr;
    hdr.frame_id = device_user_id;
    sensor_msgs::msg::Image::SharedPtr img_raw_msg, img_rect_msg;

    if(camera_info_manager->isCalibrated())
    {
      RCLCPP_INFO_ONCE(basler_node->get_logger(), "Camera is calibrated!");
    }
    else
    {
      RCLCPP_INFO_ONCE(basler_node->get_logger(), "Camera not calibrated!");
    }

    sensor_msgs::msg::CameraInfo final_cam_info = camera_info_manager->getCameraInfo();

    /* Start the grabbing... The camera device is parameterized with a
       default configuration which sets up free-running continuous acquisition.*/
    camera.StartGrabbing(Pylon::GrabStrategy_LatestImageOnly);

    // This smart pointer will receive the grab result data.
    Pylon::CGrabResultPtr ptr_grab_result;
    while(camera.IsGrabbing() && rclcpp::ok())
    {
      // Wait for an image and then retrieve it. A timeout of 5000 ms is used.
      camera.RetrieveResult(5000, ptr_grab_result, Pylon::TimeoutHandling_ThrowException);

      // Image grabbed successfully?
      if(ptr_grab_result->GrabSucceeded())
      {
        /*RCLCPP_INFO_STREAM(basler_node->get_logger(),
          "SizeX: "<< ptr_grab_result->GetWidth());
        RCLCPP_INFO_STREAM(basler_node->get_logger(),
          "SizeY: " << ptr_grab_result->GetHeight());*/
    
        // Convert the grabbed buffer to pylon imag
        format_converter.Convert(pylon_image, ptr_grab_result);
        // Create an OpenCV image out of pylon image
        open_cv_image = cv::Mat(ptr_grab_result->GetHeight(), ptr_grab_result->GetWidth(), CV_8UC3, (uint8_t *) pylon_image.GetBuffer());

        cv::resize(open_cv_image, resized_open_cv_image, cv::Size(1280, 920));

        // Create a display window
        // cv::namedWindow("OpenCV Display Window", cv::WINDOW_AUTOSIZE);
        // Display the current image with opencv
        // cv::imshow("OpenCV Display Window", resized_open_cv_image);
        hdr.stamp = basler_node->now();   
        img_raw_msg = cv_bridge::CvImage(hdr, "bgr8", resized_open_cv_image).toImageMsg();

        /// Publish raw image
        final_cam_info.header.stamp = basler_node->now();
        raw_pub.publish(*img_raw_msg, final_cam_info);
        /// Publish rectify Image
        if(camera_info_manager->isCalibrated())
        {
          pinhole_model->rectifyImage(resized_open_cv_image, open_cv_image_rect);
          img_rect_msg = cv_bridge::CvImage(hdr, "bgr8", open_cv_image_rect).toImageMsg();
          rect_pub.publish(img_rect_msg);
        }

        /// For livestreaming
        // cv::waitKey(1);
      }
      else
      {
        RCLCPP_ERROR_STREAM(basler_node->get_logger(),
          "Error: " << ptr_grab_result->GetErrorCode()
          << " " << ptr_grab_result->GetErrorDescription());
      }

      rclcpp::spin_some(basler_node);
      // loop_rate.sleep();
    }
  }
  catch(GenICam::GenericException &e)
  {
    RCLCPP_ERROR_STREAM(basler_node->get_logger(),
     "An exception occurred." << std::endl
     << e.GetDescription());
    exit_code = 1;
  }

  rclcpp::shutdown();
  return exit_code;
}
