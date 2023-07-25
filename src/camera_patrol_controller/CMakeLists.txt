cmake_minimum_required(VERSION 3.0.2)
project(camera_patrol_controller)

find_package(catkin REQUIRED
  actionlib
  rospy
  smelter_actions
  smelter_msgs
)

catkin_python_setup()

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES camera_patrol_controller
  CATKIN_DEPENDS
    actionlib
    rospy
    smelter_actions
    smelter_msgs
#  DEPENDS system_lib
)

#############
## Install ##
#############
catkin_install_python(PROGRAMS nodes/camera_patrol_controller_node.py DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY launch/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)
install(DIRECTORY config/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config)
