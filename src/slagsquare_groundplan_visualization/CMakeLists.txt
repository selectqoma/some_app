cmake_minimum_required(VERSION 3.0.2)
project(slagsquare_groundplan_visualization)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  rospy
  sensor_msgs
  smelter_msgs
  std_msgs
)

catkin_python_setup()

catkin_package(
  INCLUDE_DIRS
  LIBRARIES
  CATKIN_DEPENDS
    cv_bridge
    rospy
    sensor_msgs
    smelter_msgs
    std_msgs
  DEPENDS
)

#############
## Install ##
#############
catkin_install_python(
  PROGRAMS nodes/groundplan_visualization_node.py scripts/groundplan_demo.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)
install(DIRECTORY config/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config)

