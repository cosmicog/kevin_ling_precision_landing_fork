
#include <functional>
#include <uEye.h>
#include "WaveUEyeThread.hpp"
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>

//#define DEBUG_APRIL_LATENCY_PROFILING

// debug 
#include "opencv2/highgui/highgui.hpp"


WaveUEyeThread::WaveUEyeThread(ros::NodeHandle nh,
                               std::condition_variable& cv) :
                               main_thread_condition_variable_(cv)
{
  cameraid = 0;
  img_width = 752;
  img_height = 480;
  img_bpp = 8;
  running = false;
  camera_exposure = 0.160; //in  ms

  nh.param<int>("cameraid", cameraid, 0);
  nh.param<int>("width", img_width, 752);
  nh.param<int>("height", img_height, 480);
  nh.param<int>("left", img_left, -1);
  nh.param<int>("top", img_top, -1);
  nh.param<int>("rate", rate, 22);
  nh.param<double>("camera_exposure", camera_exposure,0.160);


  nh.param<double>("image_scaling", image_scaling_, 0.5);
  ROS_INFO("Image scaling set to %f.", image_scaling_);

  nh.param<bool>("PUBLISH_IMAGE", publish_image_, false);

  image_pub = nh.advertise<sensor_msgs::Image>("/camera/image", 1);

  running = true;
  image_capture_thread_ = std::thread(
      std::bind(&WaveUEyeThread::image_capture_loop, this));

  index_ = 0;
} 


WaveUEyeThread::~WaveUEyeThread()
{
  running = false;
}


void WaveUEyeThread::stop_thread()
{
  std::cerr << "Stopping uEye thread..." << std::endl;
  running = false;
  image_capture_thread_.join();
}


std::mutex* WaveUEyeThread::get_small_image_mutex(int index)
{
  if (index < 0 || index >= NUM_FRAMES_BUFFERED)
  {
    return NULL;
  }
  return &small_image_mutex_[index];
}


cv::Mat* WaveUEyeThread::get_small_image(int index)
{
  if (index < 0 || index >= NUM_FRAMES_BUFFERED)
  {
    return NULL;
  }
  return &small_image_[index];
}


int WaveUEyeThread::get_index_to_process(ros::Time &image_time)
{
  // Check time, if it is over 20 ms it is unlikely detection will finish before
  // the next frame is ready, so just wait.
  double dt = (ros::Time::now() - image_time_[index_to_process_]).toSec();
  if (dt > 0.02)
  {
    return -1;
  }

  image_time = image_time_[index_to_process_];

  return index_to_process_;
}


void WaveUEyeThread::image_capture_loop()
{
  std::cerr<<"Trying to open camera: "<<cameraid<<std::endl
      <<"   width: "<<img_width<<std::endl
      <<"  height: "<<img_height<<std::endl
      <<"    rate: "<<rate<<std::endl;

  HIDS hCam = cameraid;
  if(is_InitCamera (&hCam, NULL)!= IS_SUCCESS){
    ROS_FATAL("Error Initializing camera.");
    return;
  }

  SENSORINFO sensor_info;
  is_GetSensorInfo (hCam, &sensor_info);
  std::cerr<<"Sensor info: "<<std::endl
      <<"  model: " <<sensor_info.strSensorName<<std::endl
      <<"  max width: "<<sensor_info.nMaxWidth<<std::endl
      <<"  max height: "<<sensor_info.nMaxHeight<<std::endl;
    
  if((img_width < sensor_info.nMaxWidth) && (img_left == -1)) {
    std::cerr<<"Left not specified. Assuiming centering the sensor"<<std::endl;
    img_left = (sensor_info.nMaxWidth - img_width)/2;
  }
  if((img_height< sensor_info.nMaxHeight) && (img_top == -1)) {
    std::cerr<<"Top not specified. Assuiming centering the sensor"<<std::endl;
    img_top = (sensor_info.nMaxHeight - img_height)/2;
  }

  // Set up frame buffers. There are NUM_FRAMES_BUFFERED buffers that will be used in a ring,
  // done by the driver automatically.
  //
  // TODO: Clean this up. Currently removing ring buffer from driver side.
  char* imgMem[1];
  int memId[1];
  for (int i = 0; i < 1; i++)
  {
    if ( IS_SUCCESS != is_AllocImageMem(hCam, img_width, img_height, img_bpp, &imgMem[i], &memId[i]) )
    {
      std::cerr << "Error allocating image buffer." << std::endl;
    }
    if ( IS_SUCCESS != is_AddToSequence(hCam, imgMem[i], memId[i]) )
    {
      std::cerr << "Error adding image buffer to sequence." << std::endl;
    }
  }


  if ( IS_SUCCESS != is_SetColorMode (hCam, IS_CM_MONO8) )
  {
    std::cerr << "Error setting color mode..." << std::endl;
  }
  //is_SetImageSize (hCam, img_width, img_height); 
  //is_SetAOI(hCam, IS_SET_IMAGE_AOI, &img_left, &img_top, &img_width, &img_height);

  double enable = 1;
  double disable = 0;
  if ( IS_SUCCESS != is_SetAutoParameter (hCam, IS_SET_ENABLE_AUTO_GAIN, &enable, 0) )
  {
    std::cerr << "Error enabling auto gain." << std::endl;
  }
/*
  if ( IS_SUCCESS != is_SetAutoParameter (hCam, IS_SET_ENABLE_AUTO_WHITEBALANCE, &enable, 0) )
  {
    std::cerr << "Error enabling auto whitebalance." << std::endl;
  }
*/
  if ( IS_SUCCESS != is_SetAutoParameter (hCam, IS_SET_ENABLE_AUTO_FRAMERATE, &disable, 0) )
  {
    std::cerr << "Error disabling auto framerate." << std::endl;
  }
  if ( IS_SUCCESS != is_SetAutoParameter (hCam, IS_SET_ENABLE_AUTO_SHUTTER, &disable, 0) )
  {
    std::cerr << "Error disabling auto shutter." << std::endl;
  }

  double min_exposure;
  if (IS_SUCCESS != is_Exposure(hCam, IS_EXPOSURE_CMD_GET_EXPOSURE_RANGE_MIN, &min_exposure, 8)) {
    std::cerr << "Error getting min exposure time." << std::endl;
  } else {
    ROS_INFO("uEye: Min exposure is %f", min_exposure);
  }
  
  if (camera_exposure == 0) {
    if ( IS_SUCCESS != is_SetAutoParameter (hCam, IS_SET_ENABLE_AUTO_SHUTTER, &enable, 0) ) {
      std::cerr << "Error enabling auto shutter." << std::endl;
    } else {
      std::cerr << "Automatic exposure time enabled." << std::endl;
    }
  } else {
    if (camera_exposure < min_exposure) {
      camera_exposure = min_exposure;
    }

    if (IS_SUCCESS != is_Exposure(hCam, IS_EXPOSURE_CMD_SET_EXPOSURE, &camera_exposure, 8)) {
      std::cerr << "Error setting exposure time." << std::endl;
    } else {
      ROS_INFO("uEye: Exposure set to %f", camera_exposure);
    }

    double exposure = 0;
    if (IS_SUCCESS != is_Exposure(hCam, IS_EXPOSURE_CMD_GET_EXPOSURE, &exposure, 8))
    {
      std::cerr << "Error getting exposure setting." << std::endl;
    } else {
      ROS_INFO("uEye: Exposure is actually %f", exposure);
    }
  }
  
  
  // Set up continuous video free run
  if ( IS_SUCCESS != is_SetExternalTrigger(hCam, IS_SET_TRIGGER_OFF) )
  {
    std::cerr << "Error setting external trigger." << std::endl;
  }
 
  if ( IS_SUCCESS != is_EnableEvent(hCam, IS_SET_EVENT_FRAME) )
  {
    std::cerr << "Error enabling event." << std::endl;
  }

  double curr_fps = 0;  
  is_SetFrameRate(hCam, rate, &curr_fps);
  std::cerr << "CURR_FPS: " << curr_fps << std::endl;

  is_GetImageMemPitch(hCam, &img_step);
  img_data_size = img_height*img_step;

  cv_bridge::CvImage cvi;
  cvi.header.frame_id = "image";
  cvi.encoding = "mono8";

  // Start video capture
  if ( IS_SUCCESS != is_CaptureVideo(hCam, IS_WAIT) )
  {
    std::cerr << "Error starting video capture." << std::endl;
  }
  else
  {
    std::cerr << "Video capture started." << std::endl;
  }

  while (ros::ok() && running) {
    if(is_WaitEvent(hCam, IS_SET_EVENT_FRAME, 110) == IS_SUCCESS) {

      static int count = 0; // only used for debugging, but set it anyway


      //
      // Locked mutex
      //
      if (!small_image_mutex_[index_].try_lock()) {
        // This only happens if the entire ring buffer fills up before the
        // main thread is done with a frame, so something is wrong. Drop
        // the new frame.

        // Still augment this since the uEye driver will move to next buffer.
        //index_++;

        std::cerr << "Cam thread is blocked.\n";

        continue;
      }

      image_time_[index_] = ros::Time::now();

#ifdef DEBUG_APRIL_LATENCY_PROFILING
      bool print_message = 0 == (count % 25);
      if (print_message)
      {
        fprintf(stderr, "s %d at %f\n", count, ros::Time::now().toSec());
      }
#endif // DEBUG_APRIL_LATENCY_PROFILING

      cv::resize(cv::Mat(img_height, img_width, CV_8UC1, imgMem[0]),
                 small_image_[index_], cv::Size(), image_scaling_,
                 image_scaling_);

      small_image_mutex_[index_].unlock();

      //
      // unlocked mutex
      //
      
      if (publish_image_) {
        cvi.header.seq = count;
        cvi.header.stamp = ros::Time::now();
        cvi.image = small_image_[index_];
        image_pub.publish(cvi.toImageMsg());
      }

      // Wakeup the main thread
      index_to_process_ = index_;
      main_thread_condition_variable_.notify_one();


#ifdef DEBUG_APRIL_LATENCY_PROFILING
      if (print_message) {
        fprintf(stderr, "pub %d at %f\n", count, cvi.header.stamp.toSec());
      }
      count++;
#endif // DEBUG_APRIL_LATENCY_PROFILING

      index_++;
      if (index_ == NUM_FRAMES_BUFFERED) {
        index_ = 0;
      }
        
    } else {
      std::cerr << "missed!\n";
    }
  }

  // Shutdown sequence
  is_StopLiveVideo(hCam, IS_WAIT);
  is_ClearSequence(hCam);

  for (int i = 0; i < 1; i++)
  {
    is_FreeImageMem(hCam, imgMem[i], memId[i]);
  }

  is_ExitCamera(hCam);

  std::cerr << "uEye thread finished." << std::endl;


  main_thread_condition_variable_.notify_one();
}

