cmake_minimum_required(VERSION 3.0.2)
project(cascade_pot_detection)

find_package(catkin REQUIRED
  cv_bridge
  rospy
  rostest
  rosunit
  sensor_msgs
  std_msgs
)

catkin_python_setup()

catkin_package(
  INCLUDE_DIRS
  LIBRARIES
    # pot_detection
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
catkin_install_python(PROGRAMS nodes/birdview_pot_detection_node.py DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY launch/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)
install(DIRECTORY config/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config)
install(DIRECTORY media/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/media)

##########
## TEST ##
##########
# catkin_add_nosetests(test/test_pot_detection.py)
