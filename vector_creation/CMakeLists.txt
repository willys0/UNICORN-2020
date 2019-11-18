cmake_minimum_required(VERSION 2.8.3)
project(vector_creation)

# if CMAKE_BUILD_TYPE is not specified, take 'Release' as default
IF(NOT CMAKE_BUILD_TYPE)
    SET(CMAKE_BUILD_TYPE Release)
ENDIF(NOT CMAKE_BUILD_TYPE)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS 
roscpp
rospy 
geometry_msgs 
message_generation
#Peter
gazebo
#std_msgs
#genmsg
)
#message_generation

#Peter
#add_message_files(FILES Num.msg)

add_message_files(
	FILES
	vector.msg
	vectors.msg
)

#Peter
#add_service_files(DIRECTORY srv FILES AddTwoInts.srv)

generate_messages(
	DEPENDENCIES
	#Peter
	#std_msgs
	#vector.msg?
)

# Declare a catkin package
catkin_package(
	CATKIN_DEPENDS
	  message_runtime
	  roscpp
)

include_directories(include ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS})

#Peter
link_directories(${GAZEBO_LIBRARY_DIRS})
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

## Build 
add_executable(vector_creation src/vector_creation_node.cpp)
target_link_libraries(vector_creation ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES} pthread)

