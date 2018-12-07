# Quadrotor AprilTag Landing #
Source code from my master's thesis project published as-is for educational purposes. Here is a video of the final experimental results: https://www.youtube.com/watch?v=oycwswSWEB8

The source code is organized in Robot Operating System (ROS) packages. The AprilTag and vision specific source code is under vision->april_tags_ueye. The entry point is the *ros_april.cpp* file. A lot of the heavy lifting for the tag detection is done in *TagDetector.cc*. The uEye stuff in there is a USB camera driver.

External dependencies:
- http://wiki.ros.org/asctec_mav_framework

The related thesis can be found here: https://uwspace.uwaterloo.ca/handle/10012/8803