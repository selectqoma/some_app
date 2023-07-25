cmake_minimum_required(VERSION 3.0.2)
project(smelter_deployment_tests)

find_package(catkin REQUIRED COMPONENTS
  rostest
  rospy
)

catkin_package(
  CATKIN_DEPENDS
    rostest
    rospy
)

## Add tests to run
add_rostest(launch/milestone_streams_test.launch)
add_rostest(launch/osi_pi_tags_test.launch)
# add_rostest(launch/slagsquare_monitoring_system_test.launch)
