cmake_minimum_required(VERSION 2.8.3)
project(mil_dev_tools)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  nav_msgs
  geometry_msgs
)

catkin_python_setup()

catkin_package()

