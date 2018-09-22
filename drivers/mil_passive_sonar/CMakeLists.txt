cmake_minimum_required(VERSION 2.8.3)
project(mil_passive_sonar)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  message_generation
)

catkin_python_setup()

add_message_files(
  FILES
  Ping.msg
)

generate_messages(DEPENDENCIES std_msgs)


catkin_package(
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
