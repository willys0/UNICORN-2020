cmake_minimum_required(VERSION 2.8.3)
project(vector_creation)

# if CMAKE_BUILD_TYPE is not specified, take 'Release' as default
IF(NOT CMAKE_BUILD_TYPE)
    SET(CMAKE_BUILD_TYPE Release)
ENDIF(NOT CMAKE_BUILD_TYPE)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp geometry_msgs message_generation)
#message_generation

add_message_files(
	FILES
	vector.msg
	vectors.msg
)

generate_messages(
	DEPENDENCIES
	#vector.msg?
)

# Declare a catkin package
catkin_package(
	CATKIN_DEPENDS
	  message_runtime
	  roscpp
)

include_directories(include ${catkin_INCLUDE_DIRS})

## Build 
add_executable(vector_creation src/vector_creation_node.cpp)
target_link_libraries(vector_creation ${catkin_LIBRARIES})

