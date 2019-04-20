cmake_minimum_required(VERSION 2.8.3)
project(mil_usb_to_can)
find_package(catkin REQUIRED COMPONENTS
  message_generation
  rospy
  serial
)
catkin_python_setup()

add_service_files(
  FILES
  SetActuator.srv
)
generate_messages()

catkin_package()

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  add_rostest(test/adder_device.test)
endif()
