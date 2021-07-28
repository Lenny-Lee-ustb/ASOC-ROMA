cmake_minimum_required(VERSION 2.8.3)
project(localization)
find_package(catkin REQUIRED COMPONENTS
	roscpp
	rospy
	std_msgs
	message_generation
)

catkin_package(
   CATKIN_DEPENDS urdf xacro
)

