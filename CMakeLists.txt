cmake_minimum_required(VERSION 3.8)
project(ros2_ipcamera)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED)
find_package(camera_info_manager REQUIRED)
find_package(message_filters REQUIRED)

include_directories(
  include
)

set(THIS_PACKAGE_INCLUDE_DEPENDS
  rclcpp
  sensor_msgs
  std_msgs
  cv_bridge
  image_transport
  OpenCV
  camera_info_manager
)

set(SYNC_INCLUDE_DEPENDS
  rclcpp
  sensor_msgs
  message_filters
)

set(IPCAMERA_LIB_NAME ${PROJECT_NAME})
set(SYNC_LIB_NAME SYNC_NODE)

add_library(${IPCAMERA_LIB_NAME} SHARED
  src/ros2_ipcamera/ros2_ipcamera.cpp)
ament_target_dependencies(${IPCAMERA_LIB_NAME} ${THIS_PACKAGE_INCLUDE_DEPENDS})

add_executable(ros2_ipcamera_node src/ros2_ipcamera_node.cpp)
ament_target_dependencies(ros2_ipcamera_node ${THIS_PACKAGE_INCLUDE_DEPENDS})
target_link_libraries(ros2_ipcamera_node ${IPCAMERA_LIB_NAME})

add_library(${SYNC_LIB_NAME} SHARED
  src/ros2_ipcamera/message_sync.cpp)
ament_target_dependencies(${SYNC_LIB_NAME} ${SYNC_INCLUDE_DEPENDS})

add_executable(message_sync_node src/message_sync_node.cpp)
ament_target_dependencies(message_sync_node ${SYNC_INCLUDE_DEPENDS})
target_link_libraries(message_sync_node ${SYNC_LIB_NAME})

install(
  TARGETS ${IPCAMERA_LIB_NAME}
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

install(TARGETS
  ros2_ipcamera_node
  message_sync_node
  DESTINATION lib/${PROJECT_NAME}/
)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY
  launch
  config
  rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

