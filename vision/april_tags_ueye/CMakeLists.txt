cmake_minimum_required(VERSION 2.4.6)
project(april_tags_ueye)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE Release)
set(CMAKE_BUILD_TYPE Release)

find_package(catkin REQUIRED COMPONENTS
             roscpp
             cv_bridge
             sensor_msgs
             geometry_msgs
             image_transport
             tf
             wave_utils)

#########################
## OpenCV Requirements ##
#########################
find_package(OpenCV)
include_directories(${OpenCV_INCLUDE_DIRS})


########################
## Eigen Requirements ##
########################
find_package(Eigen REQUIRED)

include_directories(include 
                    ${EIGEN_INCLUDE_DIRS})
add_definitions(${EIGEN_DEFINITIONS})

set(CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS}")

###################################
## catkin specific configuration ##
###################################
catkin_package()

# I COMMENTED THESE TWO OUT NOW
#####set (CMAKE_C_FLAGS   "-Ofast -fopenmp" )
#####set (CMAKE_CXX_FLAGS "-Ofast -fopenmp" )

#set (CMAKE_CXX_FLAGS   "-march=atom -mtune=atom -Ofast " )
#set (CMAKE_CXX_FLAGS "-msse3 -ip  -no-prec-div         -parallel -O3 -fPIC" )

include_directories(include/AprilTags ${catkin_INCLUDE_DIRS})
file(GLOB libsrc
    "src/*.cc"
)

add_executable(${PROJECT_NAME}
               src/ros_april.cpp
               src/kf_generic.cpp
               ${libsrc}
               src/WaveUEyeThread.cpp)

target_link_libraries(${PROJECT_NAME}
                      ueye_api
                      ${catkin_LIBRARIES}
                      ${OpenCV_LIBRARIES})

