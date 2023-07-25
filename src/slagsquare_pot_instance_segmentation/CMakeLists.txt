cmake_minimum_required(VERSION 3.0.2)
project(slagsquare_pot_instance_segmentation)

find_package(catkin REQUIRED
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
  PROGRAMS
    nodes/pot_detector_node.py
    scripts/rosbag_to_dataset.py
    scripts/test_pot.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)
install(DIRECTORY config/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config)
