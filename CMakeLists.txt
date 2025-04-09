cmake_minimum_required(VERSION 3.8)
project(risk_guard)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(PCL REQUIRED QUIET COMPONENTS common features filters io segmentation surface)

# add_definitions(-DIMAGE_AND_PCL)
# add_definitions(-DIMAGE_COMPRESS_AND_PCL)
# add_definitions(-DONLY_IMAGE)
add_definitions(-DONLY_IMAGE_COMPRESS)

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
find_package(pcl_conversions REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(image_geometry REQUIRED)

find_package(Pylon QUIET)
if (NOT ${Pylon_FOUND} OR "$ENV{Pylon_INCLUDE_DIRS}" STREQUAL "")
  include("${CMAKE_CURRENT_SOURCE_DIR}/cmake/FindPylon.cmake")
endif()

set(msg_files
  "msg/AggregatedPerception.msg"
  "msg/AggregatedPerceptionCompressed.msg"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  DEPENDENCIES std_msgs sensor_msgs
  ADD_LINTER_TESTS
)

ament_export_dependencies(rosidl_default_runtime)

include_directories(
  include
  ${Pylon_INCLUDE_DIRS}
)

set(THIS_PACKAGE_INCLUDE_DEPENDS
  rclcpp
  sensor_msgs
  std_msgs
  cv_bridge
  image_transport
  OpenCV
  yaml-cpp
  camera_info_manager
  image_geometry
)

set(SYNC_INCLUDE_DEPENDS
  rclcpp
  cv_bridge
  OpenCV
  sensor_msgs
  message_filters
  PCL
  pcl_conversions
)

set(RISK_GUARD_LIB_NAME ros2_ipcamera_lib)
set(SYNC_LIB_NAME WRITTER_SYNC_NODE)

add_library(${RISK_GUARD_LIB_NAME} SHARED
  src/risk_guard/ip_camera.cpp
  src/risk_guard/image_transport_wrap.cpp)
ament_target_dependencies(${RISK_GUARD_LIB_NAME} ${THIS_PACKAGE_INCLUDE_DEPENDS})

add_executable(ip_camera_node src/ip_camera_node.cpp)
ament_target_dependencies(ip_camera_node ${THIS_PACKAGE_INCLUDE_DEPENDS})
target_link_libraries(ip_camera_node ${RISK_GUARD_LIB_NAME})

add_executable(image_transport_node src/image_transport_node.cpp)
ament_target_dependencies(image_transport_node ${THIS_PACKAGE_INCLUDE_DEPENDS})
target_link_libraries(image_transport_node ${RISK_GUARD_LIB_NAME})

add_executable(basler_ros_node src/basler_ros_node.cpp)
ament_target_dependencies(basler_ros_node ${THIS_PACKAGE_INCLUDE_DEPENDS})
target_link_libraries(basler_ros_node ${Pylon_LIBRARIES})

add_library(${SYNC_LIB_NAME} SHARED
  src/risk_guard/lidar_camera_writter.cpp
  src/risk_guard/setup_synchronizer.cpp)
target_link_libraries(${SYNC_LIB_NAME}
  ${PCL_LIBRARIES} ${YAML_CPP_LIBRARIES})
target_include_directories(${SYNC_LIB_NAME} PUBLIC
  ${PCL_INCLUDE_DIRS})
ament_target_dependencies(${SYNC_LIB_NAME} ${SYNC_INCLUDE_DEPENDS})

rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} rosidl_typesupport_cpp)
target_link_libraries(${SYNC_LIB_NAME} "${cpp_typesupport_target}")

add_executable(lidar_camera_writter_node src/lidar_camera_writter_node.cpp)
ament_target_dependencies(lidar_camera_writter_node ${SYNC_INCLUDE_DEPENDS})
target_link_libraries(lidar_camera_writter_node ${SYNC_LIB_NAME})

add_executable(setup_synchronizer_node src/setup_synchronizer_node.cpp)
ament_target_dependencies(setup_synchronizer_node ${SYNC_INCLUDE_DEPENDS})
target_link_libraries(setup_synchronizer_node ${SYNC_LIB_NAME})

install(
  TARGETS ${RISK_GUARD_LIB_NAME}
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

install(TARGETS
  ip_camera_node
  lidar_camera_writter_node
  setup_synchronizer_node
  image_transport_node
  basler_ros_node
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

