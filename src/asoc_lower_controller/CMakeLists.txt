cmake_minimum_required(VERSION 3.0.2)
project(asoc_lower_controller)
find_package(catkin REQUIRED COMPONENTS
  rospy
  roscpp
  std_msgs
  joy
)
catkin_package(

)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(asoc_lower_platform src/asoc_lower_platform.cpp src/include/asoc_lower_platform.hpp)
target_link_libraries(asoc_lower_platform ${catkin_LIBRARIES})
