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

add_executable(asoc_lower_controller src/asoc_lower_controller.cpp)
target_link_libraries(asoc_lower_controller ${catkin_LIBRARIES})

add_executable(asoc_lower_controller_angle src/asoc_lower_controller_angle.cpp)
target_link_libraries(asoc_lower_controller_angle ${catkin_LIBRARIES})

add_executable(asoc_lower_platform src/asoc_lower_platform.cpp src/include/asoc_lower_platform.hpp)
target_link_libraries(asoc_lower_platform ${catkin_LIBRARIES})

# add_executable(encoder_can0 src/encoder_can0.cpp src/include/asoc_lower_platform.hpp)
# target_link_libraries(encoder_can0 ${catkin_LIBRARIES})

add_executable(encoder_cantest src/encoder_cantest.cpp src/include/asoc_lower_platform.hpp)
target_link_libraries(encoder_cantest ${catkin_LIBRARIES})