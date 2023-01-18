cmake_minimum_required(VERSION 2.8.3)
project(mission_planning)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  actionlib_msgs
  geographic_msgs
  geometry_msgs
  harpia_msgs
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS actionlib_msgs geographic_msgs geometry_msgs harpia_msgs std_msgs message_runtime

)

