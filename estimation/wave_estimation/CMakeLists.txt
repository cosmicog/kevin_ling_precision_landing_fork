cmake_minimum_required(VERSION 2.8.3)
project(wave_estimation)

set(CMAKE_BUILD_TYPE Release)

find_package(catkin REQUIRED COMPONENTS
  roscpp
)

find_package(Eigen 3 QUIET)
if(NOT Eigen_FOUND)
  set(Eigen_INCLUDE_DIR "${EIGEN3_INCLUDE_DIR}")
endif()

include_directories(
  include 
  ${EIGEN_INCLUDE_DIRS}
)
add_definitions(${EIGEN_DEFINITIONS})

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES wave_estimation
  CATKIN_DEPENDS roscpp
)

###########
## Build ##
###########
include_directories(
  ${catkin_INCLUDE_DIRS}
  include
)

add_library(${PROJECT_NAME}
  src/kalman_filter_base.cc
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

