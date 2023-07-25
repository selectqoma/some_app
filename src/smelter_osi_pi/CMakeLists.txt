cmake_minimum_required(VERSION 3.0.2)
project(smelter_osi_pi)

find_package(catkin REQUIRED COMPONENTS
  roslint
  rospy
  rostest
  smelter_msgs
  smelter_srvs
  std_msgs
)

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS
    rospy
    rostest
    smelter_msgs
    smelter_srvs
    std_msgs
)

catkin_install_python(
  PROGRAMS
    nodes/osi_pi_proxy_node.py
    nodes/slagsquare_pot_states_osi_pi_publisher_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)
install(DIRECTORY config/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config)

roslint_python()

add_rostest(tests/osi_pi_proxy_test.launch)
