from launch import LaunchDescription
from launch.actions import ExecuteProcess

def generate_launch_description():

  topics = ['/F_l_cam/basler_left_driver/image_rect',
            '/F_r_cam/basler_right_driver/image_rect',
            '/points']
  split_flag = True
  n_giga = 1

  path_name = '/home/uc3m/bagfiles/'
  bagfile_name = 'test1'
  full_name = path_name + bagfile_name

  full_cmd = ['ros2', 'bag', 'record', '-o', full_name]
  if split_flag:
    split_size = n_giga * 1000000000 # In Bytes
    full_cmd.append('-b ' + str(split_size))

  full_cmd = full_cmd + topics

  # print(full_cmd)

  record_node = ExecuteProcess(
    cmd = full_cmd,
    output = 'screen'
  )

  return LaunchDescription([
    record_node
  ])
