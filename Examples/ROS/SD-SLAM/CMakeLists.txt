cmake_minimum_required(VERSION 2.8)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

rosbuild_init()

IF(NOT ROS_BUILD_TYPE)
  SET(ROS_BUILD_TYPE Release)
ENDIF()

MESSAGE("Build type: " ${ROS_BUILD_TYPE})

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall -O3 -march=native ")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -O3 -march=native -std=c++11")

find_package(OpenCV 3.0 QUIET)
if(NOT OpenCV_FOUND)
   find_package(OpenCV 2.4.3 REQUIRED)
endif()

find_package(MRPT REQUIRED poses gui vision)
find_package(Eigen3 3.1.0 REQUIRED)
find_package(Pangolin REQUIRED)
find_package(Boost COMPONENTS system REQUIRED )

include_directories(
  ${PROJECT_SOURCE_DIR}
  ${PROJECT_SOURCE_DIR}/../../../
  ${PROJECT_SOURCE_DIR}/../../../src
  ${EIGEN3_INCLUDE_DIR}
  ${Pangolin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIR}
)

set(LIBS 
  ${OpenCV_LIBS} 
  ${Pangolin_LIBRARIES}
  ${Boost_LIBRARIES}
  ${PROJECT_SOURCE_DIR}/../../../lib/libSD_SLAM.so
  ${MRPT_LIBRARIES}
)

# Node for monocular camera
rosbuild_add_executable(Monocular
  src/ros_monocular.cc
)
target_link_libraries(Monocular ${LIBS})

# Node for RGB-D camera
rosbuild_add_executable(RGBD
  src/ros_rgbd.cc
)
target_link_libraries(RGBD ${LIBS})

# Node for RGB-IMU fusion
rosbuild_add_executable(Fusion
  src/ros_fusion.cc
)
target_link_libraries(Fusion ${LIBS})



