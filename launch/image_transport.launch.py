from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, Shutdown)
from launch.substitutions import (LaunchConfiguration, TextSubstitution)
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_path

def generate_launch_description():

  sensor_params_path = LaunchConfiguration('sensor_params_path')
  sensor_params_path_arg = DeclareLaunchArgument(name='sensor_params_path',
  	default_value=TextSubstitution(text='setup.yaml'),
    description='Sensors config file with camera topics')

  image_transport_node = Node(
    package='risk_guard',
    executable='image_transport_node',
    name='image_transport_node',
    output='screen',
    parameters=[
      # Private params
      {'sensor_params': sensor_params_path},
      {'in_transport': "raw"}
    ],
    on_exit=Shutdown()
  )

  return LaunchDescription([
  	sensor_params_path_arg,
    image_transport_node
  ])  
