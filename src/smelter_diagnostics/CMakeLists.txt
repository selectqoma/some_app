cmake_minimum_required(VERSION 3.0.2)
project(smelter_diagnostics)

find_package(catkin REQUIRED
  diagnostic_aggregator
  diagnostic_msgs
  rospy
  smelter_osi_pi
  smelter_srvs
)

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS
    diagnostic_aggregator
    diagnostic_msgs
    rospy
    smelter_osi_pi
    smelter_srvs
)

catkin_install_python(
  PROGRAMS
    nodes/diagnostics_publisher_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY config/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config)
install(DIRECTORY launch/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)
