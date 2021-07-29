cmake_minimum_required(VERSION 3.0.2)
project(asoc_lower_controller)
find_package(catkin REQUIRED COMPONENTS
  rospy
  roscpp
  std_msgs
  joy
  
)
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES asoc_lower_controller
#  CATKIN_DEPENDS rospy rpscpp std_msgs
#  DEPENDS system_lib
)


include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(canTest1 src/canTest1.cpp)
target_link_libraries(canTest1 ${catkin_LIBRARIES})

add_executable(canTest2 src/canTest2.cpp)
target_link_libraries(canTest2 ${catkin_LIBRARIES})

add_executable(canTest3 src/canTest3.cpp)
target_link_libraries(canTest3 ${catkin_LIBRARIES})

add_executable(canTest4 src/canTest4.cpp)
target_link_libraries(canTest4 ${catkin_LIBRARIES})

add_executable(canTest2copy src/canTest2copy.cpp)
target_link_libraries(canTest2copy ${catkin_LIBRARIES})

add_executable(canTest3copy src/canTest3copy.cpp)
target_link_libraries(canTest3copy ${catkin_LIBRARIES})

add_executable(canTest5 src/canTest5.cpp)
target_link_libraries(canTest5 ${catkin_LIBRARIES})

add_executable(canTest_single src/canTest_single.cpp)
target_link_libraries(canTest_single ${catkin_LIBRARIES})

add_executable(canTestPD src/canTestPD.cpp)
target_link_libraries(canTestPD ${catkin_LIBRARIES})

add_executable(canTest_slim src/canTest_slim.cpp)
target_link_libraries(canTest_slim ${catkin_LIBRARIES})

add_executable(stick_control src/stick_control.cpp)
target_link_libraries(stick_control ${catkin_LIBRARIES})

add_executable(canTest2_angle src/canTest2_angle.cpp)
target_link_libraries(canTest2_angle ${catkin_LIBRARIES})

add_executable(canTest3_angle src/canTest3_angle.cpp)
target_link_libraries(canTest3_angle ${catkin_LIBRARIES})

add_executable(canTest3_8angle src/canTest3_8angle.cpp)
target_link_libraries(canTest3_8angle ${catkin_LIBRARIES})

add_executable(canTest5s src/canTest5s.cpp)
target_link_libraries(canTest5s ${catkin_LIBRARIES})

add_executable(canTest5s_body src/canTest5s_body.cpp)
target_link_libraries(canTest5s_body ${catkin_LIBRARIES})
