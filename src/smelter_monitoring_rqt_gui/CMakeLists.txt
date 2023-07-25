cmake_minimum_required(VERSION 3.0.2)
project(smelter_monitoring_rqt_gui)

find_package(catkin REQUIRED COMPONENTS
  rqt_console
  rqt_gui
  rqt_image_view
  rqt_robot_monitor
  rqt_runtime_monitor
  rqt_top
)

catkin_package()

install(DIRECTORY launch/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)
install(DIRECTORY config/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config)
