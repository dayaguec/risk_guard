from launch_ros.actions import (PushRosNamespace, Node)
from launch import LaunchDescription
from launch.actions import (IncludeLaunchDescription, DeclareLaunchArgument, GroupAction, TimerAction, Shutdown)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (LaunchConfiguration, PathJoinSubstitution,
  TextSubstitution)
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os

# Find ips: sudo arp-scan --interface=enp7s0 --localnet

def generate_launch_description():

  fisheye_config_dir = os.path.join(get_package_share_directory('ros2_ipcamera'), 'config')
  fisheye_config_file = 'file://' + os.path.join(fisheye_config_dir, "fisheye_info.yaml")

  fisheye_launch = GroupAction(
    actions = [
      IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
          PathJoinSubstitution([
            FindPackageShare('ros2_ipcamera'), 'launch', 'ipcamera.launch.py'
            ])
          ]),
          launch_arguments = {
            # Global params
            'rtsp_uri' : 'rtsp://admin:uc3m_lplp@192.168.1.132:554/profile2/media.smp',
            'image_topic' : 'fisheye_image_raw',
            'node_rate' : '25.0',
            'image_width' : '2048',
            'image_height' : '2048',
            'frame_id' : 'fisheye',
            'config_file' : fisheye_config_file
          }.items()
      )
    ]
  )

  reolink_config_dir = os.path.join(get_package_share_directory('ros2_ipcamera'), 'config')
  reolink_config_file = 'file://' + os.path.join(reolink_config_dir, "reolink_info.yaml")

  reolink_launch = GroupAction(
    actions = [
      IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
          PathJoinSubstitution([
            FindPackageShare('ros2_ipcamera'), 'launch', 'ipcamera.launch.py'
            ])
          ]),
          launch_arguments = {
            # Global params
            'rtsp_uri' : 'rtsp://admin:uc3m_lplp@192.168.1.131:554/h265Preview_01_main',
            'image_topic' : 'reolink_image_raw',
            'node_rate' : '25.0',
            'image_width' : '3840',
            'image_height' : '2160',
            'frame_id' : 'fisheye',
            'config_file' : reolink_config_file
          }.items()
      )
    ]
  )

  hick_config_dir = os.path.join(get_package_share_directory('ros2_ipcamera'), 'config')
  hick_config_file = 'file://' + os.path.join(hick_config_dir, "reolink_info.yaml")

  hick_launch = GroupAction(
    actions = [
      IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
          PathJoinSubstitution([
            FindPackageShare('ros2_ipcamera'), 'launch', 'ipcamera.launch.py'
            ])
          ]),
          launch_arguments = {
            # Global params
            'rtsp_uri' : 'rtsp://admin:montraffic2019@192.168.1.64:554/h264/ch1/main/av_stream',
            'image_topic' : 'hick_image_raw',
            'node_rate' : '20.0',
            'image_width' : '2688',
            'image_height' : '1520',
            'frame_id' : 'hickvision',
            'config_file' : hick_config_file
          }.items()
      )
    ]
  )

  ouster_launch = GroupAction(
    actions = [
      IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
          PathJoinSubstitution([
            FindPackageShare('ouster_ros'), 'launch', 'driver.launch.py'
            ])
          ]),
          launch_arguments = {
            # Global params
            'viz' : 'False'
          }.items()
      )
    ]
  )

  rviz_config_path = PathJoinSubstitution([
    FindPackageShare('ros2_ipcamera'), 'rviz', 'setup.rviz'
  ])
  rviz_node = TimerAction(
    period = 3.0,
    actions = [
      Node(
        package='rviz2',
        executable='rviz2',
        output='log',
        arguments=['-d', rviz_config_path]
      )
    ]
  )

  return LaunchDescription([
    #fisheye_launch,
    #reolink_launch,
    #ouster_launch,
    hick_launch,
    rviz_node
  ])