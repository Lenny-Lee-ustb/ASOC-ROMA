cmake_minimum_required(VERSION 3.0.2)
project(asoc_upper_controller)

find_package(catkin REQUIRED COMPONENTS
roscpp
rospy  
std_msgs
tf
dynamic_reconfigure
geometry_msgs
nav_msgs
visualization_msgs
std_msgs
tf
)

catkin_package(
  CATKIN_DEPENDS visualization_msgs
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(upper_controller_iit
src/upper_controller_iit.cpp
src/include/upper_controller.hpp)
target_link_libraries(upper_controller_iit
  ${catkin_LIBRARIES}
)

add_executable(upper_controller_angle 
src/upper_controller_angle.cpp
src/include/upper_controller.hpp)
target_link_libraries(upper_controller_angle
  ${catkin_LIBRARIES}
)

add_executable(upper_controller_anti 
src/upper_controller_anti.cpp
src/include/upper_controller.hpp)
target_link_libraries(upper_controller_anti
  ${catkin_LIBRARIES}
)
