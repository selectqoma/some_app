cmake_minimum_required(VERSION 3.0.2)
project(smelter_srvs)

find_package(catkin REQUIRED COMPONENTS message_generation smelter_msgs std_msgs)

add_service_files(DIRECTORY srv FILES JSON.srv RegisterPotMovements.srv WritePTZ.srv)

generate_messages(DEPENDENCIES smelter_msgs std_msgs)

catkin_package(CATKIN_DEPENDS message_runtime smelter_msgs std_msgs)
