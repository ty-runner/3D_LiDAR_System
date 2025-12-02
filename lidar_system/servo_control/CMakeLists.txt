cmake_minimum_required(VERSION 3.8)
project(servo_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
include_directories(${tf2_ros_INCLUDE_DIRS})
find_package(laser_geometry REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(PCL REQUIRED COMPONENTS people)
find_package(pcl_conversions REQUIRED)
find_package(visualization_msgs REQUIRED)

#add_executable(people_detector_node src/people_detector_node.cpp)
#ament_target_dependencies(rclcpp sensor_msgs visualization_msgs pcl_conversions)
#target_link_libraries(people_detector_node
#  pcl_people pcl_common pcl_io)
# Dynamic TF Broadcaster Node
add_executable(dynamic_tf_broadcaster src/dynamic_tf_broadcaster.cpp)
ament_target_dependencies(dynamic_tf_broadcaster
  rclcpp
  std_msgs
  geometry_msgs
  tf2_ros
  tf2
)

# Laser Scan to PointCloud Node
add_executable(lidar_pointcloud_node src/lidar_pointcloud_node.cpp)
ament_target_dependencies(lidar_pointcloud_node
  rclcpp
  sensor_msgs
  laser_geometry
  tf2_ros
  tf2_sensor_msgs
  tf2
)

# Scan Sequence Node
add_executable(scan_sequence_node src/scan_sequence_node.cpp)
ament_target_dependencies(scan_sequence_node
  rclcpp
  std_msgs
 
)



# Install executables
install(TARGETS
  dynamic_tf_broadcaster
  lidar_pointcloud_node
  scan_sequence_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_python_install_package(servo_control)

install(PROGRAMS
  scripts/servo_motor_node.py
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
