import os
import pathlib
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, Shutdown
from launch.substitutions import (LaunchConfiguration, TextSubstitution)
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

  rtsp_uri = LaunchConfiguration('rtsp_uri')
  rtsp_uri_arg = DeclareLaunchArgument(name='rtsp_uri', default_value=TextSubstitution(text=''),
    description='RTSP Uri for camera video streaming')
  image_topic = LaunchConfiguration('image_topic')
  image_topic_arg = DeclareLaunchArgument(name='image_topic', default_value=TextSubstitution(text='fisheye_image_raw'),
    description='Topic for ros image')
  node_rate = LaunchConfiguration('node_rate')
  node_rate_arg = DeclareLaunchArgument(name='node_rate', default_value=TextSubstitution(text='25.0'),
    description='Update rate for this node')
  image_width = LaunchConfiguration('image_width')
  image_width_arg = DeclareLaunchArgument(name='image_width', default_value=TextSubstitution(text='2048'),
    description='Image width')
  image_height = LaunchConfiguration('image_height')
  image_height_arg = DeclareLaunchArgument(name='image_height', default_value=TextSubstitution(text='2048'),
    description='Image height')
  frame_id = LaunchConfiguration('frame_id')
  frame_id_arg = DeclareLaunchArgument(name='frame_id', default_value=TextSubstitution(text='fisheye_ip_camera'),
    description='Image frame_id')
  config_file = LaunchConfiguration('config_file')
  config_file_arg = DeclareLaunchArgument(name='config_file', default_value=TextSubstitution(text=''),
    description='Image frame_id')

  ipcamera_node = Node(
    package='risk_guard',
    executable='ip_camera_node',
    name='ip_camera_node',
    output='screen',
    parameters=[
      {'rtsp_uri': rtsp_uri},
      {'image_topic': image_topic},
      {'frame_id': frame_id},
      {'node_rate': node_rate},
      {'image_width': image_width},
      {'image_height': image_height},
      {"camera_calibration_file": config_file}
    ],
    on_exit=Shutdown()
  )

  return LaunchDescription([
    rtsp_uri_arg,
    image_topic_arg,
    node_rate_arg,
    image_width_arg,
    image_height_arg,
    frame_id_arg,
    config_file_arg,
    ipcamera_node
  ])