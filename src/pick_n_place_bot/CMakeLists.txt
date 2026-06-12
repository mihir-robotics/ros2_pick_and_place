cmake_minimum_required(VERSION 3.8)
project(pick_n_place_bot)

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

# Camera Node
add_executable(camera_node src/camera_node.cpp)
target_include_directories(camera_node PRIVATE include)
ament_target_dependencies(camera_node
  rclcpp
  sensor_msgs
  cv_bridge
  OpenCV
)

# Detector Node
add_executable(detector_node src/detector_node.cpp)
target_include_directories(detector_node PRIVATE include)
ament_target_dependencies(detector_node
  rclcpp
  std_msgs
  sensor_msgs
  cv_bridge
  image_transport
  OpenCV
)

# Manipulator Node
add_executable(manipulator_node src/manipulator_node.cpp)
target_include_directories(manipulator_node PRIVATE include)
ament_target_dependencies(manipulator_node
  rclcpp
  std_msgs
)

install(TARGETS
  camera_node
  detector_node
  manipulator_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_dependencies(
  rclcpp
  std_msgs
  sensor_msgs
  cv_bridge
  image_transport
  OpenCV
)

ament_package()
