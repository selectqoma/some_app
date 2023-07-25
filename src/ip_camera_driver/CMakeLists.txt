cmake_minimum_required(VERSION 3.0.2)
project(ip_camera_driver)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  camera_calibration_parsers
  cv_bridge
  diagnostic_msgs
  rospy
  sensor_msgs
  smelter_actions
  smelter_msgs
  smelter_srvs
  std_msgs
)

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS
    actionlib
    camera_calibration_parsers
    cv_bridge
    diagnostic_msgs
    rospy
    sensor_msgs
    smelter_actions
    smelter_msgs
    smelter_srvs
    std_msgs
)

catkin_install_python(
  PROGRAMS
    nodes/flir_cgi_driver_node.py
    nodes/ip_camera_driver_node.py
    nodes/ip_camera_onvif_ptz_control_driver_node.py
    nodes/rtsp_streamer_node.py
    scripts/camera_info_publisher.py
    scripts/multifile_bag_player.py
    scripts/republish_image.py
    scripts/rosbag_image_extractor.py
    scripts/rtsp_test.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)
install(DIRECTORY config/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config)
install(DIRECTORY calibrations/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/calibrations)
