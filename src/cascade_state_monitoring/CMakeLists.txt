cmake_minimum_required(VERSION 3.0.2)
project(cascade_state_monitoring)

find_package(catkin REQUIRED
  cv_bridge
  rospy
  sensor_msgs
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
    std_msgs
  DEPENDS
)

#############
## Install ##
#############
catkin_install_python(
  PROGRAMS nodes/fumes_cover_monitoring_node.py nodes/pot_filling_monitoring_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)
install(DIRECTORY config/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config)
install(DIRECTORY media/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/media)
