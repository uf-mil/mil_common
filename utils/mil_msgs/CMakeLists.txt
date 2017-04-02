cmake_minimum_required(VERSION 2.8.3)
project(mil_msgs)
find_package(catkin
  REQUIRED COMPONENTS
    message_generation
    message_runtime
    geometry_msgs
    sensor_msgs
    actionlib
    interactive_markers
    std_msgs
    std_srvs
    actionlib_msgs
)

add_action_files(FILES
  MoveTo.action
)

add_message_files(FILES
  PoseTwistStamped.msg
  PoseTwist.msg
  VelocityMeasurements.msg
  DepthStamped.msg
  RangeStamped.msg
  VelocityMeasurement.msg
  LabeledObject.msg
  LabeledObjects.msg
)

add_service_files(FILES
  CameraToLidarTransform.srv
)

generate_messages(
  DEPENDENCIES std_msgs actionlib_msgs geometry_msgs sensor_msgs
)

catkin_package(
  DEPENDS
  CATKIN_DEPENDS 
    message_generation
    message_runtime
    geometry_msgs
    actionlib
    tf
    interactive_markers
    std_msgs
    std_srvs
    actionlib_msgs
  INCLUDE_DIRS
    include
  LIBRARIES
)

include_directories(
    include
  SYSTEM
    ${EIGEN_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIR}
    ${catkin_INCLUDE_DIRS}
)

