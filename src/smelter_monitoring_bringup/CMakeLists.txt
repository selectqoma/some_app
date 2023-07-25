cmake_minimum_required(VERSION 3.0.2)
project(smelter_monitoring_bringup)

find_package(catkin REQUIRED)
catkin_package()

#############
## Install ##
#############
install(DIRECTORY launch/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)
#install(DIRECTORY config/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config)
