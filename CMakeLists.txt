cmake_minimum_required(VERSION 2.8.3)
project(movement)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  rosserial_arduino
  message_generation
)

add_message_files(DIRECTORY msg FILES
  SpeedMotor.msg
)

generate_messages(DEPENDENCIES)

catkin_package(
  CATKIN_DEPENDS message_runtime
)

include_directories(${catkin_INCLUDE_DIRS})

#add_executable(convert2speed src/convertVel2SpeedMotor.cpp)
#target_link_libraries(convert2speed ${catkin_LIBRARIES})

catkin_install_python(PROGRAMS
  src/keyboard_vel_controller.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)