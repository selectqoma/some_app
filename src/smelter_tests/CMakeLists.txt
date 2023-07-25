cmake_minimum_required(VERSION 3.0.2)
project(smelter_tests)

find_package(catkin REQUIRED COMPONENTS
  ip_camera_driver
  rostest
  rospy
)

catkin_package(
  CATKIN_DEPENDS
    ip_camera_driver
    rostest
    rospy
)

## Add tests to run
add_rostest(launch/rtsp_test.launch)
add_rostest(launch/slagsquare_monitoring_ci_system_test.launch)
