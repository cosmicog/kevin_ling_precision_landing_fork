cmake_minimum_required(VERSION 2.8)
project(qr_apriltag_follower)

set(CMAKE_BUILD_TYPE Release)

find_package(catkin REQUIRED COMPONENTS
             roscpp tf
             asctec_hl_comm
             qr_outerloop_pid 
             wave_msgs wave_pelican wave_utils)

catkin_package()

include_directories(
   include
   ${catkin_INCLUDE_DIRS})

add_executable(${PROJECT_NAME} 
               src/apriltag_follower_node.cc)

target_link_libraries(${PROJECT_NAME}
                      ${catkin_LIBRARIES})

# Enable C++11 features.
SET(CMAKE_CXX_FLAGS "-std=c++0x")
