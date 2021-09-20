cmake_minimum_required(VERSION 3.0.2)
project(tmotor_leg)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES tmotortest
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)


include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(tmotortest 
src/tmotor_single_test.cpp
src/include/tmotor_common.hpp)
target_link_libraries(tmotortest ${catkin_LIBRARIES})

add_executable(tmotor_controller 
src/tmotor_controller.cpp
src/include/tmotor_common.hpp)
target_link_libraries(tmotor_controller ${catkin_LIBRARIES})

add_executable(tmotor_controller2 
src/tmotor_controller2.cpp
src/include/tmotor_common.hpp)
target_link_libraries(tmotor_controller2 ${catkin_LIBRARIES})

add_executable(tmotor_controller3 
src/tmotor_controller3.cpp
src/include/tmotor_common.hpp)
target_link_libraries(tmotor_controller3 ${catkin_LIBRARIES})

add_executable(tmotor_rp_test
src/tmotor_rp_test.cpp
src/include/tmotor_common.hpp)
target_link_libraries(tmotor_rp_test ${catkin_LIBRARIES})

add_executable(tmotor_controller4 
src/tmotor_controller4.cpp
src/include/tmotor_common.hpp)
target_link_libraries(tmotor_controller4 ${catkin_LIBRARIES})

add_executable(tmotor_controller5 
src/tmotor_controller5.cpp
src/include/tmotor_common.hpp)
target_link_libraries(tmotor_controller5 ${catkin_LIBRARIES})