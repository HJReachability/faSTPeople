cmake_minimum_required(VERSION 2.8.3)
project(data_logger)

find_package(catkin REQUIRED COMPONENTS
  rospy
  fastrack_msgs
  geometry_msgs
  std_msgs
)

catkin_package(
  INCLUDE_DIRS
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS
    rospy
    fastrack_msgs
    geometry_msgs
    std_msgs
  DEPENDS
)
