cmake_minimum_required(VERSION 2.8.3)
project(mission_fault_mitigation)

find_package(catkin REQUIRED COMPONENTS
  actionlib_msgs
  geographic_msgs
  geometry_msgs
  harpia_msgs
  roscpp
  rosplan_dispatch_msgs
  rosplan_knowledge_msgs
  rospy
  std_msgs
)

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   actionlib_msgs#   geographic_msgs#   geometry_msgs#   harpia_msgs#   rosplan_dispatch_msgs#   rosplan_knowledge_msgs#   std_msgs
# )


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES MissionFaultMitigation
  CATKIN_DEPENDS actionlib_msgs geographic_msgs geometry_msgs harpia_msgs roscpp rosplan_dispatch_msgs rosplan_knowledge_msgs rospy std_msgs
#  DEPENDS system_lib
)


