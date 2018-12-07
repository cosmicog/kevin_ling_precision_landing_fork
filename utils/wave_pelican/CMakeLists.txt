cmake_minimum_required(VERSION 2.8.3)

project(wave_pelican)

set(CMAKE_BUILD_TYPE Release)

find_package(catkin REQUIRED COMPONENTS roscpp asctec_hl_comm)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES wave_pelican
  CATKIN_DEPENDS roscpp asctec_hl_comm)


include_directories(
   include
   ${catkin_INCLUDE_DIRS}
)

add_library(wave_pelican src/pelican_utils.cc)

add_executable(wave_pelican_test test/wave_pelican_test.cc)
target_link_libraries(wave_pelican_test
   ${catkin_LIBRARIES}
   wave_pelican
)

