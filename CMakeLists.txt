PROJECT(dpg_slam)
CMAKE_MINIMUM_REQUIRED(VERSION 2.6)

MESSAGE(STATUS "Compiling DPG-SLAM")
MESSAGE(STATUS "Using compiler: ${CMAKE_CXX_COMPILER}")
MESSAGE(STATUS "Build Type: ${CMAKE_BUILD_TYPE}")
MESSAGE(STATUS "Arch: ${CMAKE_SYSTEM_PROCESSOR}")

SET(CMAKE_CXX_STANDARD 14)
SET(CMAKE_CXX_FLAGS "-std=c++14 -Wall")

IF(${CMAKE_BUILD_TYPE} MATCHES "Release")
  MESSAGE(STATUS "Additional Flags for Release mode")
  SET(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -fopenmp -O2 -DNDEBUG")
ELSEIF(${CMAKE_BUILD_TYPE} MATCHES "Debug")
  MESSAGE(STATUS "Additional Flags for Debug mode")
  SET(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -g")
ENDIF()

INCLUDE($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
ROSBUILD_INIT()
SET(ROS_BUILD_STATIC_LIBS true)
SET(ROS_BUILD_SHARED_LIBS false)

MESSAGE(STATUS "ROS-Overrride Build Type: ${CMAKE_BUILD_TYPE}")
MESSAGE(STATUS "CXX Flags: ${CMAKE_CXX_FLAGS}")

SET(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
SET(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#SET(libs roslib roscpp rosbag glog gflags amrl-shared-lib boost_system lua5.1
#    pthread)
SET(libs roslib roscpp rosbag glog gflags boost_system lua5.1 pthread)

ADD_LIBRARY(shared_library
            src/visualization/visualization.cc)
#            src/vector_map/vector_map.cc)

#ADD_SUBDIRECTORY(src/shared)
#INCLUDE_DIRECTORIES(src/shared)
INCLUDE_DIRECTORIES(src)
INCLUDE_DIRECTORIES(/usr/include/eigen3)

find_package(PCL 1.3 REQUIRED)

INCLUDE_DIRECTORIES(src)
INCLUDE_DIRECTORIES(/usr/include/eigen3)
INCLUDE_DIRECTORIES(${PCL_INCLUDE_DIRS})
LINK_DIRECTORIES(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

ROSBUILD_ADD_EXECUTABLE(dpg_slam
                        src/dpg_slam/dpg_slam_main.cc
        src/dpg_slam/dpg_slam.cc src/dpg_slam/dpg_node.cc src/dpg_slam/math_utils.cc)
TARGET_LINK_LIBRARIES(dpg_slam shared_library ${libs} tbb gtsam ${PCL_LIBRARIES})

ROSBUILD_ADD_EXECUTABLE(dpg_data_runner src/runner/dpg_data_runner_main.cc)
TARGET_LINK_LIBRARIES(dpg_data_runner shared_library ${libs})

