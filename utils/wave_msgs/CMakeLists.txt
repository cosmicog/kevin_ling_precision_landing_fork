cmake_minimum_required(VERSION 2.4.6)
project (wave_msgs)

find_package(catkin REQUIRED COMPONENTS std_msgs message_generation)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
set(ROS_BUILD_TYPE RelWithDebInfo)

add_message_files(
   FILES
   PidLog.msg
   QRAltitudePid.msg
   QRControl.msg
   QRPosition.msg
   QRPositionPid.msg
   QRStateEstimate.msg
   QRRelativeHeight.msg
   QRYawPid.msg
   qr_state.msg
)

generate_messages(DEPENDENCIES std_msgs)

catkin_package(CATKIN_DEPENDS message_runtime)
