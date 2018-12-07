cmake_minimum_required(VERSION 2.8.3)
project(qr_outerloop_pid)


set(CMAKE_BUILD_TYPE Release)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
             roscpp
             std_msgs
             sensor_msgs
             geometry_msgs
             asctec_hl_comm
             wave_control
             wave_msgs
             wave_pelican
             wave_utils)

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
add_service_files(FILES SetTranslationControllerStatus.srv)
generate_messages(DEPENDENCIES std_msgs)
catkin_package(CATKIN_DEPENDS message_runtime)

###########
## Build ##
###########

include_directories(include
                    ${catkin_INCLUDE_DIRS})

add_executable(pelican_outerloop_pid
               src/pelican_outerloop_pid_node.cc
               src/outerloop_pid_controller.cc
               src/height_controller.cc
               src/yaw_controller.cc
               src/translation_controller.cc)

target_link_libraries(pelican_outerloop_pid
                      ${catkin_LIBRARIES}
                      wave_utils
                      wave_pelican
                      wave_control)

