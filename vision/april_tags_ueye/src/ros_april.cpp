#include <iostream>
#include <ctype.h>
#include <sstream>
#include <math.h>
#ifndef PI
const double PI = 3.14159265358979323846;
#endif
const double TWOPI = 2.0*PI;



#include "WaveUEyeThread.hpp"

#include <condition_variable>
#include <mutex>
#include <thread>


#include "opencv2/video/tracking.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"

#include <sensor_msgs/Image.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/tf.h>
#include <cv_bridge/cv_bridge.h>

#include "ros/ros.h"
#include <tf/tf.h>

#include <wave_utils/wave_math.h>
#include <wave_utils/wave_math.h>


// April tags detector and various families that can be selected by command line option
#include "AprilTags/TagDetector.h"
#include "AprilTags/Tag16h5.h"
//#include "AprilTags/Tag25h7.h"
//#include "AprilTags/Tag25h9.h"
//#include "AprilTags/Tag36h9.h"
//#include "AprilTags/Tag36h11.h"

//#define DEBUG_CAMERA_ROTATED_90_DEGREES_RIGHT
//#define DEBUG_KF_TEST
//#define DEBUG_ROS_APRIL
//#define DEBUG_APRIL_PROFILING
//#define DEBUG_APRIL_LATENCY_PROFILING

ros::Publisher pose_pub;
ros::Publisher failed_detection_image_publisher;
bool publish_failed_detections = false;
#define IMAGE_SCALING 0.5f // Run at half width and height, or 1/4 resolution


ros::Time image_time;

double m_tagSize;
//double m_tagSize(0.154); // Tag on 8.5 x 11 paper
//double m_tagSize(0.343); // Large foam core one

bool disable_cropping = false;
int cropped_detection_fail_count = 0;


/*
// Parameters for ueyecamera on Pelican, with 0.68X wide (macro) angle lens attachment
// at quarter resolution.
double m_fx(379.451971);
double m_fy(379.950068);
double m_px(184.117217);
double m_py(129.179852);
cv::Vec4f __distParam(-0.614591, 0.368921, -0.008932, -0.010523);
*/

// Parameters for ueyecamera with fisheye lens (0.28x) at quarter resolution
/*
double m_fx(268.968354);
double m_fy(268.921107);
double m_px(181.128234);
double m_py(124.293735);
cv::Vec4f __distParam(-0.457979, 0.152024, 0.001359, -0.002135);
*/

// Parameters for the zimea 184 degree fisheye lens
double m_fx(316.553467);
double m_fy(317.502499);
double m_px(352.566979);
double m_py(256.666132);
cv::Vec4f __distParam(-0.251972, 0.047324, -0.004253, 0.000091);


#define REL_WINDOW_SIZE 3.0f // Window size in multiples of m_tagSize

// Parameters for Kevin's Webcam
/*
double m_fx(612.134425 * IMAGE_SCALING);
double m_fy(607.853801 * IMAGE_SCALING);
double m_px(311.906765 * IMAGE_SCALING);
double m_py(214.333245 * IMAGE_SCALING);
cv::Vec4f __distParam(-0.041685,-0.082760,0.005497,0.008019);
*/

// Condition variable for signalling from the WaveUEyeThread.
std::condition_variable main_thread_condition_variable;
std::mutex main_thread_mutex; // To go with the condition_variable.

AprilTags::TagCodes m_tagCodes = AprilTags::tagCodes16h5;

AprilTags::TagDetector* m_tagDetector;


geometry_msgs::Point window_centre_point; // Last tag pose in camera frame
bool crop_image = false;
ros::Time prev_t;

bool use_original_detection_algorithm = false;
int tag_id_;

// Extrinsic parameters for ueye camera to pelican quad CoG.
Eigen::Matrix4d T_cam2body;


/**
 * Normalize angle to be within the interval [-pi,pi].
 */
inline double standardRad(double t) {
  if (t >= 0.) {
    t = fmod(t+PI, TWOPI) - PI;
  } else {
    t = fmod(t-PI, -TWOPI) + PI;
  }
  return t;
}

void wRo_to_euler(const Eigen::Matrix3d& wRo, double& yaw, double& pitch, double& roll) {
  yaw = standardRad(atan2(wRo(1,0), wRo(0,0)));
  double c = cos(yaw);
  double s = sin(yaw);
  pitch = standardRad(atan2(-wRo(2,0), wRo(0,0)*c + wRo(1,0)*s));
  roll  = standardRad(atan2(wRo(0,2)*s - wRo(1,2)*c, -wRo(0,1)*s + wRo(1,1)*c));
}


void print_detection(AprilTags::TagDetection& detection) {
  if(detection.id != tag_id_) {
    return;
  }

  // recovering the relative pose of a tag:
  // NOTE: for this to be accurate, it is necessary to use the
  // actual camera parameters here as well as the actual tag size
  // (m_fx, m_fy, m_px, m_py, m_tagSize)

  Eigen::Vector3d tag_translation; //in camera frame
  Eigen::Matrix3d tag_rotation;    //in camera frame
  detection.getRelativeTranslationRotation(m_tagSize, m_fx, m_fy, m_px, m_py,
                                           tag_translation, tag_rotation);

  double tag_roll, tag_pitch, tag_yaw;

  // Check if this rotation is in the right order
  wRo_to_euler(tag_rotation, tag_yaw, tag_pitch, tag_roll);

  // Note: Roll measures 180 degrees at a neutral attitude for some reason.
  tag_roll = wave_utils::unwrapAngle(tag_roll + M_PI);
  Eigen::Vector3d qr_frame_tag_angles(tag_roll, tag_pitch, tag_yaw);

  Eigen::Vector4d tag_in_camera_frame(tag_translation(0), tag_translation(1),
                                      tag_translation(2), 1);


  Eigen::Vector4d tag_in_qr_body_frame = T_cam2body*tag_in_camera_frame;

//  qr_frame_tag_angles = rotation_roll_pi * qr_frame_tag_angles;
//  tag_translation = rotation_roll_pi * tag_translation;

//  qr_frame_tag_angles = rotation_yaw_half_pi * qr_frame_tag_angles;
//  tag_translation = rotation_yaw_half_pi * tag_translation;


  // TODO: Fix this to use proper 3-2-1 euler to quaternion conversions
  tf::Quaternion tag_quaternion;
  tag_quaternion.setEulerZYX(qr_frame_tag_angles(2), qr_frame_tag_angles(1),
                        qr_frame_tag_angles(0));

/*
  ROS_INFO("Tag Attitude: R:%f, P:%f, Y:%f",
           qr_frame_tag_angles(0) * 180.0 / M_PI,
           qr_frame_tag_angles(1) * 180.0 / M_PI,
           qr_frame_tag_angles(2) * 180.0 / M_PI);
*/

//  Eigen::Vector3d copterTranslation = cToM*tag_translation;
  geometry_msgs::PoseStamped to_publish;
  to_publish.header.stamp = ros::Time::now();
  to_publish.pose.position.x = tag_in_qr_body_frame(0);
  to_publish.pose.position.y = tag_in_qr_body_frame(1);
  to_publish.pose.position.z = tag_in_qr_body_frame(2);
//  tf::quaternionTFToMsg(tag_quaternion, to_publish.orientation);
// THIS IS WRONG AND CHEATING
  to_publish.pose.orientation.x = tag_roll;
  to_publish.pose.orientation.y = tag_pitch;
  to_publish.pose.orientation.z = tag_yaw;
  to_publish.pose.orientation.w = 0;

  prev_t = ros::Time::now();
  crop_image = true;

  pose_pub.publish(to_publish);


#ifdef DEBUG_ROS_APRIL
/*
  double DEG2RAD = 180.0 / M_PI;
  tf::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);
  ROS_INFO("Euler Angles: %f, %f, %f", roll*DEG2RAD, pitch*DEG2RAD, yaw*DEG2RAD);
*/
#endif
}


// Converts a coordinate in meters in the camera frame into the image's pixel coordinates.
// The returned point is not guaranteed to be in the image.
// @return point corresponding with the specified coordinate in the pixel frame.
cv::Point reproject_camera_frame_to_image_frame(double cf_x, double cf_y, double cf_z)
{
//  double x_prime = cf_y / cf_z;
//  double y_prime = cf_x / cf_z;
  double x_prime = cf_x / cf_z;
  double y_prime = cf_y / cf_z;


  double r_squared = pow(x_prime, 2) + pow(y_prime, 2);

  double x_prime2 = x_prime*(1 + __distParam(0)*r_squared + __distParam(1)*pow(r_squared,2)) +
        2*__distParam(2)*x_prime*y_prime + __distParam(3)*(r_squared + 2*pow(x_prime,2));
  double y_prime2 = y_prime*(1 + __distParam(0)*r_squared + __distParam(1)*pow(r_squared,2)) +
        __distParam(2)*(r_squared + 2*pow(y_prime,2)) + 2*__distParam(3)*x_prime*y_prime;

  return cv::Point(m_fx*x_prime2 + m_px, m_fy*y_prime2 + m_py);
}


// Returns a vector of two points, the top left corner of the ROI, then the bottom right corner of
// the ROI. Must maintain this order.
vector<cv::Point> calculate_roi_from_pose_estimate(geometry_msgs::Point tag_pose)
{
  vector<cv::Point> rect_corners;

  Eigen::Vector4d tag_position(tag_pose.x, tag_pose.y, tag_pose.z, 1);

  tag_position = T_cam2body.transpose() * tag_position;

  double x = tag_position(0);
  double y = tag_position(1);
  double z = tag_position(2);

/*
 * Code for a window size relative to the tag.
 *
  // Now do the inverse calculation. Taken from OpenCV Camera Calibration page.
  rect_corners.push_back(
        reproject_camera_frame_to_image_frame(x + m_tagSize*REL_WINDOW_SIZE, 
        y + m_tagSize*REL_WINDOW_SIZE, z) );
  
  rect_corners.push_back(
        reproject_camera_frame_to_image_frame(x - m_tagSize*REL_WINDOW_SIZE,
        y - m_tagSize*REL_WINDOW_SIZE, z) );
  
*/

  // Fixed window size
  cv::Point centre = reproject_camera_frame_to_image_frame(x, y, z);

  // TODO: Unhardcode resolutions
  rect_corners.push_back(cv::Point(centre.x - 188, centre.y - 120));
  rect_corners.push_back(cv::Point(centre.x + 188, centre.y + 120));

  return rect_corners;
}

// This function fixes the roi boundary points in place, or signals if the ROI
// cannot be fixed because too much of it is off the screen.
// Input: The vector returned by calculate_roi_from_pose_estimate()
// Return: true if the ROI is good or corrected to be good, false otherwise
bool fix_roi_boundaries(cv::Mat& image, vector<cv::Point>& roi_points)
{
  assert(2 == roi_points.size());

  // Check that upper left corner is not beyond the right and bottom bounds
  // of the image
  if ( (roi_points[0].x > image.cols)
        || (roi_points[0].y > image.rows) )
  {
    return false;
  }

  // Check that upper right corner is not beyond the left and top bounds
  // of the image
  if ( (roi_points[1].x < 0)
        || (roi_points[1].y < 0) )
  {
    return false;
  }

  // Check initial size, we want to make sure the ROI does not shrink by more
  // than some threshold percentage.
  int dx_pre = abs(roi_points[1].x - roi_points[0].x);
  int dy_pre = abs(roi_points[1].y - roi_points[0].y);


  // Fix upper left corner of ROI to be at least the upper left corner of image
  if ( roi_points[0].x < 0 )
  {
    roi_points[0].x = 0;
  }
  if ( roi_points[0].y < 0 )
  {
    roi_points[0].y = 0;
  }

  // Fix bottom right corner of ROI to be at least the bottom right corner of image
  if ( roi_points[1].x >= image.cols )
  {
    roi_points[1].x = image.cols - 1;
  }
  if ( roi_points[1].y >= image.rows )
  {
    roi_points[1].y = image.rows - 1;
  }

  // Check dimensions and make sure it didn't shrink too much
  int dx_post = abs(roi_points[1].x - roi_points[0].x);
  int dy_post = abs(roi_points[1].y - roi_points[0].y);
  if ( ( ( (float) dx_post / (float) dx_pre ) < 0.85f ) 
        || ( ( (float) dy_post / (float) dy_pre ) < 0.85f ) )
  {
    return false;
  }
  
  return true;
}


bool new_tag_pos_prediction = false;

void tag_pos_pred_cb(geometry_msgs::PointStamped msg)
{
  new_tag_pos_prediction = true;
  window_centre_point = msg.point;
}


// TODO: This is currently a dirty, filthy, no-good duplicate of the callback.
void run_april_tag_detection_and_processing(cv::Mat& image_gray)
{
#ifdef DEBUG_APRIL_LATENCY_PROFILING
  // Need a new way to profile
#endif //DEBUG_APRIL_LATENCY_PROFILING

#ifdef DEBUG_APRIL_PROFILING
  static int n_count = 0;
  static double t_accum = 0;

  ros::Time start = ros::Time::now();

  if ( (ros::Time::now() - prev_t).toSec() > 1.0 )
  {
    crop_image = false;
  }
#endif // DEBUG_APRIL_PROFILING

#ifdef DEBUG_ROS_APRIL
  bool curr_frame_cropped = false;
#endif

  vector<AprilTags::TagDetection> detections;
  vector<cv::Point> rect_corners;

  if (!new_tag_pos_prediction || cropped_detection_fail_count >= 2 || disable_cropping)
  {
    crop_image = false;
    cropped_detection_fail_count = 0;
  }
  // Clear flag so we don't window the same place again
  new_tag_pos_prediction = false;

  if (crop_image)
  {
    rect_corners = calculate_roi_from_pose_estimate(window_centre_point);
    crop_image = fix_roi_boundaries(image_gray, rect_corners);
  }

  if (crop_image)
  {
    cv::Mat imageROI = image_gray(cv::Rect(rect_corners[0], rect_corners[1]));

#ifdef DEBUG_ROS_APRIL
    curr_frame_cropped = true;
    imshow("ROI", imageROI); // OpenCV call
    cv::waitKey(30);
#endif 

    detections= m_tagDetector->extractTags(image_gray, &imageROI, rect_corners[0].x, rect_corners[0].y);
  }
  else
  {
    if (use_original_detection_algorithm) {
      detections = m_tagDetector->extractTagsOriginal(image_gray);
    } else {
      detections= m_tagDetector->extractTags(image_gray, NULL, 0, 0);
    }
  }

  for(int i = 0; i < (int) detections.size(); i++)
  {
    print_detection(detections[i]);
  }

  if (0 == detections.size() && crop_image) {
    cropped_detection_fail_count++;

    if (publish_failed_detections) {
      static ros::Time last_failed_detection_publish_time(0);
      ros::Time current_time = ros::Time::now();

      if ((current_time - last_failed_detection_publish_time).toSec() > 0.5) {
        if (crop_image) {
          cv::rectangle(image_gray, rect_corners[0], rect_corners[1], cv::Scalar(0,255,255), 3);
        }

        cv_bridge::CvImage cvi;
        cvi.header.frame_id = "image";
        cvi.encoding = "mono8";
        cvi.header.stamp = image_time;
        cvi.image = image_gray;
        failed_detection_image_publisher.publish(cvi.toImageMsg());
        last_failed_detection_publish_time = current_time;
      }
    }
  }
#ifdef DEBUG_ROS_APRIL
  for (int i=0; i < (int) detections.size(); i++) {
    // also highlight in the image
    if(detections[i].id == 0)
    {
      detections[i].draw(image_gray);
    }
  }

  if (curr_frame_cropped)
  {
    cv::rectangle(image_gray, rect_corners[0], rect_corners[1], cv::Scalar(0,255,255), 3);
  }

  imshow("AprilResult", image_gray); // OpenCV call
  cv::waitKey(30);
#endif

#ifdef DEBUG_APRIL_PROFILING
  ros::Time end = ros::Time::now();
  n_count++;
  t_accum += (end - start).toSec();
  if (n_count >= 100)
  {
    ROS_DEBUG("Avg april tag run time: %f", t_accum/100.0);
    std::cerr << "Avg april tag run time: " << t_accum/100.0 << std::endl;
    n_count = 0;
    t_accum = 0;
  }
#endif // DEBUG_APRIL_PROFILING

#ifdef DEBUG_APRIL_LATENCY_PROFILING
  // Need a new way to profile
#endif //DEBUG_APRIL_LATENCY_PROFILING

}

void image_callback(const sensor_msgs::ImageConstPtr& picture)
{
#ifdef DEBUG_APRIL_LATENCY_PROFILING
  bool print_message = 0 == (picture->header.seq % 25);
  if (print_message)
  {
    fprintf(stderr, "rx tx was at %f\n", picture->header.stamp.toSec());
    fprintf(stderr, "rx %d at %f\n", picture->header.seq, ros::Time::now().toSec());
    //std::cerr << "rx tx was at " << picture->header.stamp.toSec() << std::endl;
    //std::cerr << "rx " << picture->header.seq << " at " << ros::Time::now().toSec() << std::endl;
  }
#endif //DEBUG_APRIL_LATENCY_PROFILING

#ifdef DEBUG_APRIL_PROFILING
  static int n_count = 0;
  static double t_accum = 0;

  ros::Time start = ros::Time::now();

  if ( (ros::Time::now() - prev_t).toSec() > 1.0 )
  {
    crop_image = false;
  }
#endif // DEBUG_APRIL_PROFILING

#ifdef DEBUG_ROS_APRIL
  bool curr_frame_cropped = false;
#endif

  // Received image should already be grayscale and at quarter resolution
  cv_bridge::CvImageConstPtr bridge = cv_bridge::toCvShare(picture,"mono8");
  
  cv::Mat image_gray(bridge->image);

  vector<AprilTags::TagDetection> detections;
  vector<cv::Point> rect_corners;

  if (!new_tag_pos_prediction)
  {
    crop_image = false;
  }
  // Clear flag so we don't window the same place again
  new_tag_pos_prediction = false;

  if (crop_image)
  {
    rect_corners = calculate_roi_from_pose_estimate(window_centre_point);
    crop_image = fix_roi_boundaries(image_gray, rect_corners);
  }

  if (crop_image)
  {
    cv::Mat imageROI = image_gray(cv::Rect(rect_corners[0], rect_corners[1]));

#ifdef DEBUG_ROS_APRIL
    curr_frame_cropped = true;
    imshow("ROI", imageROI); // OpenCV call
    cv::waitKey(30);
#endif 

    detections= m_tagDetector->extractTags(image_gray, &imageROI, rect_corners[0].x, rect_corners[0].y);
  }
  else
  {
    if (use_original_detection_algorithm) {
      detections = m_tagDetector->extractTagsOriginal(image_gray);
    } else {
      detections= m_tagDetector->extractTags(image_gray, NULL, 0, 0);
    }
  }


  for(int i = 0; i < (int) detections.size(); i++)
  {
    print_detection(detections[i]);
  }

#ifdef DEBUG_ROS_APRIL
  for (int i=0; i < (int) detections.size(); i++) {
    // also highlight in the image
    if(detections[i].id == 0)
    {
      detections[i].draw(image_gray);
    }
  }

  if (curr_frame_cropped)
  {
    cv::rectangle(image_gray, rect_corners[0], rect_corners[1], cv::Scalar(0,255,255), 3);
  }

  imshow("AprilResult", image_gray); // OpenCV call
  cv::waitKey(30);
#endif

#ifdef DEBUG_APRIL_PROFILING
  ros::Time end = ros::Time::now();
  n_count++;
  t_accum += (end - start).toSec();
  if (n_count >= 100)
  {
    ROS_DEBUG("Avg april tag run time: %f", t_accum/100.0);
    std::cerr << "Avg april tag run time: " << t_accum/100.0 << std::endl;
    n_count = 0;
    t_accum = 0;
  }
#endif // DEBUG_APRIL_PROFILING

#ifdef DEBUG_APRIL_LATENCY_PROFILING
  static double tot_t_accum = 0;
  tot_t_accum += (ros::Time::now() - picture->header.stamp).toSec();

  if (print_message)
  {
    fprintf(stderr, "done %f averate t %f\n", ros::Time::now().toSec(), tot_t_accum / 25);
//    std::cerr << "done " << ros::Time::now().toSec() << " average t ",  tot_t_accum / 25 << std::endl;
    tot_t_accum = 0;
  }
#endif //DEBUG_APRIL_LATENCY_PROFILING

  return;
}


int main( int argc, char** argv )
{
  //ROS init
  ros::init(argc, argv, "ros_april");
  ros::NodeHandle n("~");

  bool use_default_camera = true;

  // Does not disable ueye camera if not using it's message... need to fix that.
  std::string image_topic;
  if(ros::names::remap("image") == "image")
  {
    ROS_INFO("image has not been mapped, using default /camera/image");
    image_topic = std::string("/camera/image");
  }
  else
  {
    use_default_camera = false;
    image_topic = std::string(ros::names::remap("image"));
  }

  if (n.getParam("tag_size", m_tagSize)) {
    ROS_INFO("Tag size set to %f.", m_tagSize);
  } else {
    m_tagSize = 0.154;
    ROS_INFO("Tag size not set. Defaulting to %f.", m_tagSize);
  }

  n.param("disable_cropping", disable_cropping, false);
  if (disable_cropping) {
    ROS_INFO("Cropping disabled!");
  }

  n.param("use_original_detection_algorithm", use_original_detection_algorithm, false);
  if (use_original_detection_algorithm) {
    ROS_INFO("Using original april tag detection algorithm.");
  }

  if (n.getParam("tag_id", tag_id_)) {
    ROS_INFO("Tag id set to %f.", tag_id_);
  } else {
    tag_id_ = 0;
    ROS_INFO("Tag id not set. Defaulting to %f.", 0);
  }

  n.param("publish_failed_detections", publish_failed_detections, false);
  if (publish_failed_detections) {
    ROS_INFO("Publishing failed april tag detections.");
  }
  failed_detection_image_publisher = n.advertise<sensor_msgs::Image>("/wave/failed_detections", 1);

  m_tagDetector = new AprilTags::TagDetector(m_tagCodes);

  pose_pub = n.advertise<geometry_msgs::PoseStamped>("/wave/tag", 1);

  ros::Subscriber image_sub = n.subscribe(image_topic, 1, image_callback,ros::TransportHints().tcpNoDelay());
 
#ifdef DEBUG_KF_TEST
  ros::Subscriber tag_kf_pred_sub = n.subscribe("/wave/tag_pos_pred", 1, tag_pos_pred_cb, ros::TransportHints().tcpNoDelay());
#else
  ros::Subscriber tag_kf_pred_sub = n.subscribe("/wave/tag_window", 1, tag_pos_pred_cb, ros::TransportHints().tcpNoDelay());
#endif

#ifdef DEBUG_CAMERA_ROTATED_90_DEGREES_RIGHT
 T_cam2body << -1, 0, 0, 0,
                0, 1, 0, 0,
                0, 0, -1, -0.15,
                0, 0, 0, 1;
#else
  T_cam2body << 0, -1, 0, 0,
               -1, 0, 0, 0,
                0, 0, -1, -0.15,
                0, 0, 0, 1;
#endif


  WaveUEyeThread cam_thread(n, main_thread_condition_variable);

  prev_t = ros::Time::now();

  ROS_INFO("Starting tag detection.");

  while (ros::ok())
  {
    {
      std::unique_lock<std::mutex> lock(main_thread_mutex);

      main_thread_condition_variable.wait(lock);
    }

    int frame_index = cam_thread.get_index_to_process(image_time);
    if (-1 != frame_index)
    {
      std::mutex* frame_mutex = cam_thread.get_small_image_mutex(frame_index);
      if (NULL == frame_mutex)
      {
        std::cerr << "Skip processing frame...\n";
        continue;
      }
      
      std::lock_guard<std::mutex> small_image_lock(*frame_mutex);
      
      cv::Mat* image = cam_thread.get_small_image(frame_index);
      if (image)
      {
        run_april_tag_detection_and_processing(*image);
      }
    }

    ros::spinOnce();
  }

  cam_thread.stop_thread();

  return 0;
}

