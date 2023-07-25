cmake_minimum_required(VERSION 3.0.2)
project(smelter_actions)

find_package(catkin REQUIRED COMPONENTS
  actionlib_msgs
  geometry_msgs
  message_generation
  message_runtime
  smelter_msgs
  std_msgs
)

add_action_files(
  FILES
    DetectObjects.action
    Move.action
    PTZ.action
    TakeSnapshot.action
    UpdatePotStates.action
)

generate_messages(
  DEPENDENCIES
    actionlib_msgs
    geometry_msgs
    smelter_msgs
    std_msgs
)

catkin_package(
  CATKIN_DEPENDS
    actionlib_msgs
    geometry_msgs
    smelter_msgs
    std_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
