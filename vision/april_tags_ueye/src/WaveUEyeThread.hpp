/**
 * Header file for the wavelab ueye nodelet. The nodelet spits out video frames
 * from the UEye camera on the Asctec Pelican.
 *
 * Kevin Ling
 * WAVELab, University of Waterloo
 * Nov. 2013
 */

#ifndef __WAVE_UEYE_NODELET_HPP__
#define __WAVE_UEYE_NODELET_HPP__

#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <thread>
#include <condition_variable>
#include <mutex>
#include "opencv2/imgproc/imgproc.hpp"

// Size of camera ring buffer
#define NUM_FRAMES_BUFFERED 4


class WaveUEyeThread
{
public:
  WaveUEyeThread(ros::NodeHandle nh, std::condition_variable& cv);

  ~WaveUEyeThread();
  void stop_thread();
  
  // Returns a pointer to the mutex for the scaled down image at the specified
  // index. The index argument must be in the range [0, NUM_FRAMES_BUFFERED).
  std::mutex* get_small_image_mutex(int index);

  // Returns a pointer to the scaled down image at the specified index. The
  // index argument must be in the range [0, NUM_FRAMES_BUFFERED).
  cv::Mat* get_small_image(int index);

  // Returns the next frame that should be processed if the frame is not too
  // stale. Returns -1 if too much time has elapsed and it is better to wait
  // for the next frame.
  int get_index_to_process(ros::Time &image_time);

private:
  int cameraid;
  int img_width;
  int img_height;
  int img_bpp;
  int img_step;
  int img_data_size;
  int rate;
  int img_left;
  int img_top;
  double camera_exposure;

  // Factor by which to scale the image. 1 means no scaling.
  double image_scaling_;

  // Index of the latest frame in the ring buffer.
  int index_;

  // Same as index, but set just awakening the main thread so it is guaranteed
  // to have valid data. Always.
  int index_to_process_;

  // Array of already resized images. These will be accessed by the main thread.
  cv::Mat small_image_[NUM_FRAMES_BUFFERED];

  // Mutexes to control access to the small images.
  std::mutex small_image_mutex_[NUM_FRAMES_BUFFERED];

  // Array of times that the images were taken at.
  ros::Time image_time_[NUM_FRAMES_BUFFERED];


  // Flag to indicate if the image should be published to a ros message.
  bool publish_image_;

  // Flag to indicate if the thread is running. When transitioning to false,
  // it will take a bit of time before the thread actually stops.
  bool running;
  ros::Publisher image_pub;

  std::condition_variable& main_thread_condition_variable_;

protected:
  void image_capture_loop();

  // Main ROS loop

  std::thread image_capture_thread_;
};

#endif // __WAVE_UEYE_NODELET_HPP__

