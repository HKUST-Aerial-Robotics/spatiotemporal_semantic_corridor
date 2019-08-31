cmake_minimum_required(VERSION 2.8)
project(ssc_planner)

set(CMAKE_VERBOSE_MAKEFILE "true")
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11 -march=native -g")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -Wall")

find_package(catkin REQUIRED COMPONENTS
    roscpp
    visualization_msgs
    sensor_msgs)

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

find_package(OpenMP)
if (OPENMP_FOUND)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    set (CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()

find_package(common REQUIRED)
find_package(semantic_map_manager REQUIRED)
find_package(behavior_planner REQUIRED)
find_package(pubg_planner REQUIRED)
#only for debugging purpose
find_package(onlane_motion_planner REQUIRED)
#if this catkin packge's header is used by other packages, use catkin_package to
#declare the include directories of this package.
catkin_package(
#	    INCLUDE_DIRS include
)

include_directories(
    inc
    ${catkin_INCLUDE_DIRS}
    ${common_INCLUDE_DIRS}
    ${semantic_map_manager_INCLUDE_DIRS}
    ${behavior_planner_INCLUDE_DIRS}
    ${pubg_planner_INCLUDE_DIRS}
    ${onlane_motion_planner_INCLUDE_DIRS}
)

add_library(ssc_planner_lib STATIC
    src/ssc_planner/ssc_planner.cc
    src/ssc_planner/ssc_map.cc
    src/ssc_planner/map_adapter.cc
)
target_link_libraries(ssc_planner_lib
    ${common_LIBRARIES}
    semantic_map_manager
)

add_library(ssc_server_ros STATIC
    src/ssc_planner/ssc_server_ros.cc
    src/ssc_planner/ssc_visualizer.cc
)
target_link_libraries(ssc_server_ros
    ssc_planner_lib
    semantic_map_ros
)

add_executable(test_ssc_planner_with_smm
    src/test_ssc_planner_with_smm.cc
)
target_link_libraries(test_ssc_planner_with_smm
   ${catkin_LIBRARIES}
   ssc_server_ros
   behavior_planner_ros
   onlane_server_ros
)

add_executable(test_ssc_with_pubg
    src/test_ssc_with_pubg.cc
)
target_link_libraries(test_ssc_with_pubg
   ${catkin_LIBRARIES}
   ssc_server_ros
   pubg_planner_ros
   onlane_server_ros
)

#install the hearder files so that other packages can include.
#install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
 #  DESTINATION ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_INCLUDE_DESTINATION}/
 #  FILES_MATCHING PATTERN "*.h"
 #  PATTERN ".svn" EXCLUDE
#)
