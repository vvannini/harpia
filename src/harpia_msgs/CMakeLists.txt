cmake_minimum_required(VERSION 2.8.3)
project(harpia_msgs)          
find_package(catkin REQUIRED COMPONENTS
  actionlib_msgs                
  std_msgs                      
  message_generation
  geographic_msgs      
  geometry_msgs
  mavros_msgs    

)

add_service_files(
  FILES
  MissionFaultMitigation.srv
  PathPlanning.srv
  MissionGoalManager.srv
  WeatherCheck.srv
)
  
add_action_files(
  FILES
  MissionPlanner.action
)
add_message_files(
  FILES
  Tuple2D.msg
  Camera.msg
  Battery.msg
  Frame.msg
  UAV.msg
  Region.msg
  RegionPoint.msg
  Map.msg
  Goal.msg
  Mission.msg
  ChangeMission.msg
  DronePose.msg
  SafetyParams.msg
)

generate_messages(
  DEPENDENCIES
  actionlib_msgs
  std_msgs
  geographic_msgs    
  geometry_msgs  
  mavros_msgs    

)
catkin_package(
  CATKIN_DEPENDS actionlib_msgs std_msgs message_runtime 
)
include_directories(
  ${catkin_INCLUDE_DIRS}
)
