cmake_minimum_required(VERSION 2.8.3)
project(mil_passive_sonar)

find_package(catkin
  REQUIRED COMPONENTS
    rospy
    message_generation
    message_runtime
    tf2
)

add_service_files(
  FILES
    GetPulseHeading.srv
    EstimatePingerPosition.srv
    ResetPositionEstimate.srv
    StartBagging.srv
    SaveBag.srv
    SetFrequency.srv
)

catkin_python_setup()

generate_messages(DEPENDENCIES std_msgs)

catkin_package(
  CATKIN_DEPENDS
    message_generation
    message_runtime
    tf2
)

include_directories(${catkin_INCLUDE_DIRS}
)
