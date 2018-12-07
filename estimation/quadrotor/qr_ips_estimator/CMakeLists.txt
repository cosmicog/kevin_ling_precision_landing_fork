cmake_minimum_required(VERSION 2.8.3)
project(qr_ips_estimator)

find_package(catkin REQUIRED COMPONENTS
             roscpp
             geometry_msgs
             asctec_hl_comm
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
catkin_package()

###########
## Build ##
###########
include_directories(include
                    ${catkin_INCLUDE_DIRS})

add_executable(pelican_ips_estimator_node
               src/pelican_ips_estimator_node.cc
               src/ips_differentiator.cc)

target_link_libraries(pelican_ips_estimator_node
                      ${catkin_LIBRARIES})

