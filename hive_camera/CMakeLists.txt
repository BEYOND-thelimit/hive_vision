cmake_minimum_required(VERSION 3.8)
project(hive_camera)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(PCL REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)

add_executable(image_stitching_node src/image_stitching.cpp)
ament_target_dependencies(image_stitching_node rclcpp sensor_msgs message_filters OpenCV cv_bridge)

add_executable(rs_image_stitching_node src/rs_image_stitching.cpp)
ament_target_dependencies(rs_image_stitching_node rclcpp sensor_msgs message_filters OpenCV cv_bridge)

add_executable(depth_sub src/depth_sub.cpp)
ament_target_dependencies(depth_sub 
  rclcpp 
  std_msgs
  nav_msgs
  sensor_msgs 
  PCL 
  pcl_conversions 
  OpenCV
)

#node
install(TARGETS
  image_stitching_node
  rs_image_stitching_node
  depth_sub
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
