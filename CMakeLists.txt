cmake_minimum_required(VERSION 2.8.3)
project(movement)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  rosserial_arduino
)

catkin_package(CATKIN_DEPENDS)

include_directories(${catkin_INCLUDE_DIRS})

add_executable(control_directly src/controlDirectly.cpp)
target_link_libraries(control_directly ${catkin_LIBRARIES})