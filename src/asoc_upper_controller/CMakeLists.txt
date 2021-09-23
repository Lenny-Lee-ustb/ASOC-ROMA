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

generate_dynamic_reconfigure_options(
  cfg/upper_controller.cfg
)

catkin_package(
  CATKIN_DEPENDS visualization_msgs
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)


add_executable(upper_controller_angle 
src/upper_controller_angle.cpp
src/include/upper_controller.hpp)
target_link_libraries(upper_controller_angle
  ${catkin_LIBRARIES}
)

add_executable(upper_controller_xbox 
src/upper_controller_xbox.cpp
src/include/upper_controller.hpp)
target_link_libraries(upper_controller_xbox
  ${catkin_LIBRARIES}
)

add_executable(upper_controller_anti_rec 
src/upper_controller_anti_rec.cpp
src/include/upper_controller_v2.hpp)
target_link_libraries(upper_controller_anti_rec
  ${catkin_LIBRARIES}
)

add_executable(upper_controller_noturn 
src/upper_controller_noturn.cpp
src/include/upper_controller_v2.hpp)
target_link_libraries(upper_controller_noturn
  ${catkin_LIBRARIES}
)

add_executable(upper_controller_turnstraight 
src/upper_controller_turnstraight.cpp
src/include/upper_controller_v2.hpp)
target_link_libraries(upper_controller_turnstraight
  ${catkin_LIBRARIES}
)
