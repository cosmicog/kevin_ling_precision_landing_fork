cmake_minimum_required(VERSION 2.8.3)
project(wave_control)

set(CMAKE_BUILD_TYPE Release)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  wave_msgs
  wave_utils
)

find_package(Eigen REQUIRED)

include_directories(
  include 
  ${EIGEN_INCLUDE_DIRS}
)
add_definitions(${EIGEN_DEFINITIONS})

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES wave_control
  CATKIN_DEPENDS roscpp wave_msgs wave_utils
)

###########
## Build ##
###########
include_directories(
  ${catkin_INCLUDE_DIRS}
  include
  wave_utils
  wave_msgs
)

add_library(wave_control
  src/pid_controller_base.cc
)

target_link_libraries(wave_control
  ${catkin_LIBRARIES}
  wave_utils
)

#############
## CPPLint ##
#############
#FILE(GLOB LINT_SRCS src/*.cc include/*/*.h)
#add_custom_target(${PROJECT_NAME}_cpplint
#                  COMMAND cpplint --filter=-whitespace/line_length ${LINT_SRCS})


