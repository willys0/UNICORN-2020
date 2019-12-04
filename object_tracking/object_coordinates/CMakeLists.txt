cmake_minimum_required(VERSION 2.8.3)
project(object_coordinates)

# if CMAKE_BUILD_TYPE is not specified, take 'Release' as default
IF(NOT CMAKE_BUILD_TYPE)
    SET(CMAKE_BUILD_TYPE Release)
ENDIF(NOT CMAKE_BUILD_TYPE)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp geometry_msgs nav_msgs sensor_msgs darknet_ros_msgs cmake_modules message_filters tf)
find_package(Eigen REQUIRED)
#message_generation

#add_message_files(
#	FILES
#	vector.msg
#	vectors.msg
#)

#generate_messages(
#	DEPENDENCIES
	
#)

# Declare a catkin package
catkin_package(
	CATKIN_DEPENDS
	  message_runtime
	DEPENDS
	  Eigen
)

include_directories(
  ${object_coordinates}/src
  include
  ${Boost_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
)

## Build 
add_executable(object_coordinates src/object_coordinates_node.cpp)
target_link_libraries(object_coordinates ${catkin_LIBRARIES})

