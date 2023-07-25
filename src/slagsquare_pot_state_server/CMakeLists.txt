cmake_minimum_required(VERSION 3.0.2)
project(slagsquare_pot_state_server)

find_package(catkin REQUIRED
  cv_bridge
  nav_msgs
  rospy
  sensor_msgs
  smelter_actions
  smelter_msgs
  std_msgs
)

catkin_python_setup()

catkin_package(
  INCLUDE_DIRS
  LIBRARIES
  CATKIN_DEPENDS
    cv_bridge
    nav_msgs
    rospy
    sensor_msgs
    smelter_actions
    smelter_msgs
    std_msgs
  DEPENDS
)

#############
## Install ##
#############
catkin_install_python(
  PROGRAMS nodes/pot_state_server_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)
install(DIRECTORY config/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config)
install(DIRECTORY media/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/media)

##########
## TEST ##
##########
catkin_add_nosetests(
  test/test_crane_observer.py
  test/test_crane_reader.py
  test/test_crane_var.py
  test/test_GridModel.py
  WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/test)
