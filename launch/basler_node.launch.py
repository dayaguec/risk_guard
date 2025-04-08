from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, Shutdown)
from launch.substitutions import (LaunchConfiguration, TextSubstitution)
from launch_ros.actions import Node

def generate_launch_description():

  node_name = LaunchConfiguration('node_name')
  node_name_arg = DeclareLaunchArgument('node_name',
  	default_value='basler_ros_node', description='Name of the basler node')
  serial_number = LaunchConfiguration('serial_number')
  serial_number_arg = DeclareLaunchArgument(name='serial_number', default_value=TextSubstitution(text=''),
    description='Serial number of the Basler camera')
  device_user_id = LaunchConfiguration('device_user_id')
  device_user_id_arg = DeclareLaunchArgument(name='device_user_id', default_value=TextSubstitution(text='basler'),
    description='Basler Camera device id')
  node_rate = LaunchConfiguration('node_rate')
  node_rate_arg = DeclareLaunchArgument(name='node_rate', default_value=TextSubstitution(text='30.0'),
    description='Rate of the publisher node')
  camera_info_url = LaunchConfiguration('camera_info_url')
  camera_info_url_arg = DeclareLaunchArgument(name='camera_info_url', default_value=TextSubstitution(text=''),
    description='Camera info url for rectify images')

  basler_ros_node = Node(
    package='risk_guard',
    executable='basler_ros_node',
    name=node_name,
    output='screen',
    parameters=[
      {'serial_number': serial_number},
      {'device_user_id': device_user_id},
      {'node_rate': node_rate},
      {'camera_info_url': camera_info_url}
    ],
    on_exit=Shutdown()
  )

  return LaunchDescription([
  	serial_number_arg,
  	device_user_id_arg,
  	node_rate_arg,
  	camera_info_url_arg,
  	node_name_arg,
    basler_ros_node
  ])