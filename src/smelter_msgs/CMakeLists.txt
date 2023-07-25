cmake_minimum_required(VERSION 3.0.2)
project(smelter_msgs)

find_package(catkin REQUIRED COMPONENTS
    geometry_msgs
    message_generation
    sensor_msgs
    std_msgs
)

add_message_files(
  FILES
    ClassIds.msg
    ContourArray.msg
    ContourArrayStamped.msg
    Contour.msg
    DetectedObjects.msg
    PanTiltZoom.msg
    Point2DArray.msg
    Point2DArrayStamped.msg
    Point2D.msg
    Point2DStamped.msg
    PotStateArray.msg
    PotState.msg
    RectArray.msg
    RectArrayStamped.msg
    Rect.msg
    SyncedFramePosition.msg
    TagValueArray.msg
    TagValueArrayStamped.msg
    TagValue.msg
    TagValueStamped.msg
)

generate_messages(
  DEPENDENCIES
    geometry_msgs
    sensor_msgs
    std_msgs
)

catkin_package(
 CATKIN_DEPENDS
   geometry_msgs
   message_runtime
   sensor_msgs
   std_msgs
)
