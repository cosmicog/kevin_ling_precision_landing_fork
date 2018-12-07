cmake_minimum_required(VERSION 2.4.6)
project(qr_relative_target_estimator)

#set(ROS_BUILD_TYPE Debug)
set(CMAKE_BUILD_TYPE Release)

find_package(catkin REQUIRED COMPONENTS
             roscpp
             std_msgs
             geometry_msgs
             asctec_hl_comm
             wave_msgs
             wave_pelican
             wave_utils
             wave_estimation
             px_comm)

########################
## Eigen Requirements ##
########################
find_package(Eigen REQUIRED)

include_directories(include 
                    ${EIGEN_INCLUDE_DIRS})
add_definitions(${EIGEN_DEFINITIONS})

###################################
## catkin specific configuration ##
###################################
catkin_package()

###########
## Build ##
###########
include_directories(include
                    ${catkin_INCLUDE_DIRS})

add_executable(${PROJECT_NAME}
               src/relative_position_estimator_main.cc
               src/height_estimator.cc
               src/relative_height_estimator.cc
               src/target_position_estimator.cc)

target_link_libraries(${PROJECT_NAME}
                      ${catkin_LIBRARIES})

