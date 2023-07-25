cmake_minimum_required(VERSION 3.0.2)
project(slagsquare_monitoring_fsm)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  rospy
  smach
  smach_msgs
  smach_ros
  smelter_actions
  smelter_msgs
  std_msgs
)

catkin_python_setup()

###################################
## catkin specific configuration ##
###################################
catkin_package(
  # INCLUDE_DIRS include
  LIBRARIES slagsquare_monitoring_fsm
  CATKIN_DEPENDS
    actionlib
    actionlib_msgs
    rospy
    smach
    smach_msgs
    smach_ros
    smelter_actions
    smelter_msgs
    std_msgs
  # DEPENDS system_lib
)

#############
## Install ##
#############
catkin_install_python(
  PROGRAMS
    nodes/slagsquare_monitoring_fsm_node.py
    scripts/update_monitor.py
    scripts/update_timer.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)
