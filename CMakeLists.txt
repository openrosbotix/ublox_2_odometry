cmake_minimum_required(VERSION 3.5)
project(ublox_2_odometry)


find_package(ament_cmake REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ublox_msgs REQUIRED)
find_package(robot_localization REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)


## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  include/ublox_2_odometry
  include ${Boost_INCLUDE_DIRS}
)

 add_executable(ublox_2_odometry 
     src/ublox_2_odometry.cpp
     src/main.cpp )

ament_target_dependencies(ublox_2_odometry rclcpp 
  nav_msgs
  sensor_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  ublox_msgs
  robot_localization)

install(TARGETS
  ublox_2_odometry
  DESTINATION lib/${PROJECT_NAME})
   
install(DIRECTORY
  DESTINATION share/${PROJECT_NAME}/
  )

ament_export_include_directories(include)
ament_export_dependencies(std_msgs)
ament_package()