from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, Shutdown)
from launch.substitutions import (LaunchConfiguration, TextSubstitution)
from launch_ros.actions import Node

def generate_launch_description():

  camera_left_topic = LaunchConfiguration('camera_left_topic')
  camera_left_topic_arg = DeclareLaunchArgument(name='camera_left_topic', default_value=TextSubstitution(text='/F_l_cam/basler_left_driver/image_rect/compressed'),
    description='Topic for ros camera left image')
  camera_right_topic = LaunchConfiguration('camera_right_topic')
  camera_right_topic_arg = DeclareLaunchArgument(name='camera_right_topic', default_value=TextSubstitution(text='/F_r_cam/basler_right_driver/image_rect/compressed'),
    description='Topic for camera right image')
  lidar_topic = LaunchConfiguration('lidar_topic')
  lidar_topic_arg = DeclareLaunchArgument(name='lidar_topic', default_value=TextSubstitution(text='/points'),
    description='Topic for ros pointcloud')
  aggregated_perception_topic = LaunchConfiguration('aggregated_perception_topic')
  aggregated_perception_topic_arg = DeclareLaunchArgument(name='aggregated_perception_topic', default_value=TextSubstitution(text='/all_in_one'),
    description='Topic for synch message of the three topics')

  setup_synchronizer_node = Node(
    package='risk_guard',
    executable='setup_synchronizer_node',
    name='setup_synchronizer_node',
    output='screen',
    parameters=[
      {'camera_left_topic': camera_left_topic},
      {'camera_right_topic': camera_right_topic},
      {'lidar_topic': lidar_topic},
      {'aggregated_perception_topic': aggregated_perception_topic}
    ],
    on_exit=Shutdown()
  )

  return LaunchDescription([
  	camera_left_topic_arg,
  	camera_right_topic_arg,
  	lidar_topic_arg,
  	aggregated_perception_topic_arg,
    setup_synchronizer_node
  ])