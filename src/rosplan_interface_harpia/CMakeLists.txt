cmake_minimum_required(VERSION 2.8.3)
project(rosplan_interface_harpia)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  actionlib
  actionlib_msgs
  std_msgs
  std_srvs
  diagnostic_msgs
  rosplan_planning_system
  roslib
  rostest
  harpia_msgs
)

###################################
## catkin specific configuration ##
###################################

## Declare things to be passed to dependent projects
catkin_package(
  INCLUDE_DIRS include ${catkin_INCLUDE_DIRS}
  LIBRARIES
  CATKIN_DEPENDS roscpp actionlib std_msgs std_srvs diagnostic_msgs
                 rosplan_planning_system roslib rostest
  DEPENDS
)

###########
## Build ##
###########

include_directories(include)
include_directories(${catkin_INCLUDE_DIRS})

# find_package(FLEX REQUIRED)

# Disable deprecated declarations warning (about std::auto_ptr)
# add_definitions(-Wno-deprecated-declarations)

## Declare action interface for RPHarpiaExecutor
add_executable(RPHarpiaExecutor src/RPHarpiaExecutor.cpp)
add_dependencies(RPHarpiaExecutor ${catkin_EXPORTED_TARGETS})
target_link_libraries(RPHarpiaExecutor ${catkin_LIBRARIES})

#############
## Install ##
#############

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  PATTERN ".svn" EXCLUDE
)
