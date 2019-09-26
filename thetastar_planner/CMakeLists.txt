cmake_minimum_required(VERSION 2.8.3)
project(thetastar_planner)

find_package(catkin REQUIRED
        COMPONENTS
            roscpp
            tf
            nav_core
            costmap_2d
            base_local_planner
            pluginlib
        )

include_directories(
    include 
    ${catkin_INCLUDE_DIRS}
    )
add_definitions(${EIGEN3_DEFINITIONS})

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES thetastar_planner
    CATKIN_DEPENDS
        roscpp
        pluginlib
        costmap_2d
        base_local_planner
        nav_core
)

add_library(thetastar_planner src/thetastar_planner.cpp src/ThetaStar4Grid.cpp src/MinSearchHeap.cpp)
add_dependencies(thetastar_planner ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(thetastar_planner
    ${catkin_LIBRARIES}
    )

install(TARGETS thetastar_planner
       ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
       LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
       )

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  PATTERN ".svn" EXCLUDE
)

install(FILES bgp_plugin.xml
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)


