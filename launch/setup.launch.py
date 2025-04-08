from launch_ros.actions import (SetRemap, PushRosNamespace, Node)
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

  fisheye_config_dir = os.path.join(get_package_share_directory('risk_guard'), 'config')
  fisheye_config_file = 'file://' + os.path.join(fisheye_config_dir, "fisheye_info.yaml")
  fisheye_launch = GroupAction(
    actions = [
      IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
          PathJoinSubstitution([
            FindPackageShare('risk_guard'), 'launch', 'ipcamera.launch.py'
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

  reolink_config_dir = os.path.join(get_package_share_directory('risk_guard'), 'config')
  reolink_config_file = 'file://' + os.path.join(reolink_config_dir, "reolink_info.yaml")
  reolink_launch = GroupAction(
    actions = [
      IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
          PathJoinSubstitution([
            FindPackageShare('risk_guard'), 'launch', 'ipcamera.launch.py'
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

  hick_config_dir = os.path.join(get_package_share_directory('risk_guard'), 'config')
  hick_config_file = 'file://' + os.path.join(hick_config_dir, "reolink_info.yaml")
  hick_launch = GroupAction(
    actions = [
      IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
          PathJoinSubstitution([
            FindPackageShare('risk_guard'), 'launch', 'ipcamera.launch.py'
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

  ouster_params = PathJoinSubstitution([
    FindPackageShare('risk_guard'), 'config', 'ouster.yaml'
  ])
  ouster_launch = GroupAction(
    actions = [
      IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
          PathJoinSubstitution([
            FindPackageShare('ros2_ouster'), 'launch', 'driver_launch.py'
            ])
          ]),
          launch_arguments = {
            # Global params
            'params_file' : ouster_params
          }.items()
      )
    ]
  )

  basler_right_params = PathJoinSubstitution([
    FindPackageShare('risk_guard'), 'config', 'basler_right.yaml'
  ])
  basler_right_launch = GroupAction(
    actions = [
      SetRemap(src='/F_r_cam/basler_right_driver/image_rect',
        dst='/right_image'),
      IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
          PathJoinSubstitution([
            FindPackageShare('pylon_ros2_camera_wrapper'), 'launch', 'pylon_ros2_camera.launch.py'
            ])
          ]),
          launch_arguments = {
            # Global params
            'config_file' : basler_right_params,
            'node_name' : 'basler_right_driver',
            'enable_status_publisher' : 'False',
            'enable_current_params_publisher' : 'False',
            'mtu_size' : '1500',
            'camera_id' : 'F_r_cam'
          }.items()
      )
    ]
  )

  basler_left_params = PathJoinSubstitution([
    FindPackageShare('risk_guard'), 'config', 'basler_left.yaml'
  ])
  basler_left_launch = GroupAction(
    actions = [
      SetRemap(src='/F_l_cam/basler_left_driver/image_rect',
        dst='/left_image'),
      IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
          PathJoinSubstitution([
            FindPackageShare('pylon_ros2_camera_wrapper'), 'launch', 'pylon_ros2_camera.launch.py'
            ])
          ]),
          launch_arguments = {
            # Global params
            'config_file' : basler_left_params,
            'node_name' : 'basler_left_driver',
            'enable_status_publisher' : 'False',
            'enable_current_params_publisher' : 'False',
            'mtu_size' : '1500',
            'camera_id' : 'F_l_cam'
          }.items()
      )
    ]
  )

  basler_own_right_launch = GroupAction(
    actions = [
      IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
          PathJoinSubstitution([
            FindPackageShare('risk_guard'), 'launch', 'basler_node.launch.py'
            ])
          ]),
          launch_arguments = {
            # Global params
            'node_name' : 'basler_right_driver',
            'serial_number' : '23107262',
            'device_user_id' : 'F_r_cam',
            'node_rate' : '35.0',
            'camera_info_url' : 'file:///home/weasfas/ros2_ws/src/risk_guard/config/calibration/basler_right_info.yaml'
          }.items()
      )
    ]
  )

  basler_own_left_launch = GroupAction(
    actions = [
      IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
          PathJoinSubstitution([
            FindPackageShare('risk_guard'), 'launch', 'basler_node.launch.py'
            ])
          ]),
          launch_arguments = {
            # Global params
            'node_name' : 'basler_left_driver',
            'serial_number' : '23107274',
            'device_user_id' : 'F_l_cam',
            'node_rate' : '35.0',
            'camera_info_url' : 'file:///home/weasfas/ros2_ws/src/risk_guard/config/calibration/basler_left_info.yaml'
          }.items()
      )
    ]
  )

  camera_params = PathJoinSubstitution([
    FindPackageShare('risk_guard'), 'config', 'setup.yaml'
  ])
  image_transport_launch = GroupAction(
    actions = [
      IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
          PathJoinSubstitution([
            FindPackageShare('risk_guard'), 'launch', 'image_transport.launch.py'
            ])
          ]),
          launch_arguments = {
            # Global params
            'sensor_params_path' : camera_params
          }.items()
      )
    ]
  )

  setup_sync_launch = GroupAction(
    actions = [
      IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
          PathJoinSubstitution([
            FindPackageShare('risk_guard'), 'launch', 'setup_synchronizer.launch.py'
            ])
          ]),
          launch_arguments = {
            # Global params
            'camera_left_topic' : '/left_image/image_it/compressed',
            'camera_right_topic' : '/right_image/image_it/compressed',
            'lidar_topic' : '/points',
            'aggregated_perception_topic' : '/all_in_one'
          }.items()
      )
    ]
  )

  rviz_config_path = PathJoinSubstitution([
    FindPackageShare('risk_guard'), 'rviz', 'setup.rviz'
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

  # ori: image_width: 2048 and image_height: 1539

  return LaunchDescription([
    # fisheye_launch,
    # reolink_launch,
    # ouster_launch,
    # hick_launch,
    # basler_left_launch,
    # basler_right_launch,
    basler_own_right_launch,
    basler_own_left_launch,
    # image_transport_launch,
    # setup_sync_launch,
    # rviz_node
  ])