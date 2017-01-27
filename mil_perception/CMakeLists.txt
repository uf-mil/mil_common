cmake_minimum_required(VERSION 2.8.3)
project(mil_perception)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 ")

find_package(catkin
  REQUIRED COMPONENTS
    rospy
)

catkin_package(
  INCLUDE_DIRS 
    include
  LIBRARIES 
    mil_vision_lib
  CATKIN_DEPENDS
    roscpp
)

include_directories(include
  ${catkin_INCLUDE_DIRS}
)

add_library(mil_vision_lib
   src/mil_vision_lib/cv_tools.cpp
)

target_link_libraries(mil_vision_lib
   ${catkin_LIBRARIES}
   ${OpenCV_LIBS}
)

install(TARGETS mil_vision_lib
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
