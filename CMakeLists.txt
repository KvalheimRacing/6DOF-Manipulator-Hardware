cmake_minimum_required(VERSION 2.8.3)
project(crustcrawler_hardware)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wpedantic -Werror -Wfloat-equal -Wno-unused -Wold-style-cast")

find_package(Boost REQUIRED)

find_package(catkin REQUIRED COMPONENTS
	controller_manager
	diagnostic_updater
	dynamixel_sdk
	hardware_interface
	roscpp
	roslaunch
	std_msgs
	)

catkin_package(
	CATKIN_DEPENDS
		controller_manager
		hardware_interface
		roscpp
		std_msgs
	INCLUDE_DIRS include
	LIBRARIES ${PROJECT_NAME}
	)

roslaunch_add_file_check(launch)

link_directories(
	${catkin_LIBRARY_DIRS}
	)

# By separating the following includes we will only get warnings for our code
include_directories(
	SYSTEM
	${catkin_INCLUDE_DIRS}
	${Boost_INCLUDE_DIRS}
	)
include_directories(include)


add_executable(hardware_interface
	src/controller_node.cpp
	src/crustcrawler.cpp
	src/dynamixel.cpp)
target_link_libraries(hardware_interface ${catkin_LIBRARIES})
