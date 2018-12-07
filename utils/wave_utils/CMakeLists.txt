cmake_minimum_required(VERSION 2.8)
project(wave_utils)

set(CMAKE_BUILD_TYPE Release)

find_package(catkin REQUIRED COMPONENTS roscpp tf)
find_package(Eigen REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp tf
  DEPENDS Eigen)

include_directories(
   include
   ${catkin_INCLUDE_DIRS}
   ${EIGEN_INCLUDE_DIRS})

add_definitions(${EIGEN_DEFINITIONS})

add_library(${PROJECT_NAME}
            src/wave_dsp.cpp
            src/wave_math.cpp
            src/wave_string.cpp)

add_executable(${PROJECT_NAME}_test test/wave_utils_tests.cpp)

target_link_libraries(${PROJECT_NAME}_test
                      ${PROJECT_NAME}
                      ${catkin_LIBRARIES})

