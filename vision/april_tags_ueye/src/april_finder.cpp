/**
 * @file april_tags.cpp
 * @brief Example application for April tags library
 * @author: Michael Kaess
 *
 * Opens the first available camera (typically a built in camera in a
 * laptop) and continuously detects April tags in the incoming
 * images. Detections are both visualized in the live image and shown
 * in the text console. Optionally allows selecting of a specific
 * camera in case multiple ones are present and specifying image
 * resolution as long as supported by the camera. Also includes the
 * option to send tag detections via a serial port, for example when
 * running on a Raspberry Pi that is connected to an Arduino.
 */

using namespace std;

#include <iostream>
#include <cstring>
#include <vector>
#include <sys/time.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <ros/ros.h>
#include <tf/tf.h>
#include <geometry_msgs/Pose.h>


const string usage = "\n"
  "Usage:\n"
  "  apriltags_demo [OPTION...] [deviceID]\n"
  "\n"
  "Options:\n"
  "  -h  -?          Show help options\n"
  "  -a              Arduino (send tag ids over serial port)\n"
  "  -d              disable graphics\n"
  "  -C <bbxhh>      Tag family (default 36h11)\n"
  "  -F <fx>         Focal length in pixels\n"
  "  -W <width>      Image width (default 640, availability depends on camera)\n"
  "  -H <height>     Image height (default 480, availability depends on camera)\n"
  "  -S <size>       Tag size (square black frame) in meters\n"
  "  -E <exposure>   Manually set camera exposure (default auto; range 0-10000)\n"
  "  -G <gain>       Manually set camera gain (default auto; range 0-255)\n"
  "  -B <brightness> Manually set the camera brightness (default 128; range 0-255)\n"
  "\n";

const string intro = "\n"
    "April tags test code\n"
    "(C) 2012-2013 Massachusetts Institute of Technology\n"
    "Michael Kaess\n"
    "\n";


#ifndef __APPLE__
#define EXPOSURE_CONTROL // only works in Linux
#endif

#ifdef EXPOSURE_CONTROL
#include <libv4l2.h>
#include <linux/videodev2.h>
#include <fcntl.h>
#include <errno.h>
#endif

// OpenCV library for easy access to USB camera and drawing of images
// on screen
#include "opencv2/opencv.hpp"

// April tags detector and various families that can be selected by command line option
#include "AprilTags/TagDetector.h"
#include "AprilTags/Tag16h5.h"
#include "AprilTags/Tag25h7.h"
#include "AprilTags/Tag25h9.h"
#include "AprilTags/Tag36h9.h"
#include "AprilTags/Tag36h11.h"


// Needed for getopt / command line options processing
#include <unistd.h>
extern int optind;
extern char *optarg;


const char* window_name = "apriltags_demo";


// utility function to provide current system time (used below in
// determining frame rate at which images are being processed)
double tic() {
  struct timeval t;
  gettimeofday(&t, NULL);
  return ((double)t.tv_sec + ((double)t.tv_usec)/1000000.);
}


#include <cmath>

#ifndef PI
const double PI = 3.14159265358979323846;
#endif
const double TWOPI = 2.0*PI;

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

//Determines the camera pose wrt to april tag, given the april tag pose wrt camera
void getCameraPoseWRTAprilTag(const Eigen::Matrix4d& tagWRTCamera,
                              Eigen::Vector3d& trans,
                              Eigen::Matrix3d& rot) 
{
  // converting from camera frame (z forward, x right, y down) to
  // object frame (x forward, y left, z up)
  Eigen::Matrix4d M;
  M <<
    0,  0, 1, 0,
    -1, 0, 0, 0,
    0, -1, 0, 0,
    0,  0, 0, 1;
  Eigen::Matrix4d MT = M*tagWRTCamera;
  trans = MT.inverse().col(3).head(3);
  rot = tagWRTCamera.block(0,0,3,3);
}

class Demo {

  AprilTags::TagDetector* m_tagDetector;
  AprilTags::TagCodes m_tagCodes;

  ros::NodeHandle n;
  ros::Subscriber sub;
  image_transport::ImageTransport it;
  image_transport::Publisher debug_pub; //for debugging
  ros::Publisher pose_pub; //Publish pose of april tag

  bool m_draw; // draw image and April tag detections?
  bool m_arduino; // send tag detections to serial port?

  int m_width; // image size in pixels
  int m_height;
  double m_tagSize; // April tag side length in meters of square black frame
  double m_fx; // camera focal length in pixels
  double m_fy;
  double m_px; // camera principal point
  double m_py;

  int m_deviceId; // camera id (in case of multiple cameras)
  cv::VideoCapture m_cap;

  int m_exposure;
  int m_gain;
  int m_brightness;

public:

  // default constructor
  Demo() :
    // default settings, most can be modified through command line options (see below)
    m_tagDetector(NULL),
    m_tagCodes(AprilTags::tagCodes36h11),

    m_draw(true),
    m_arduino(false),

    m_width(640),
    m_height(480),
    m_tagSize(0.1524), //A4
    m_fx(707), // for microsoft
    m_fy(707), // for microsoft
    m_px(m_width/2),
    m_py(m_height/2),

    m_exposure(-1),
    m_gain(-1),
    m_brightness(-1),

    m_deviceId(0),
    it(n)
  {
  
    debug_pub = it.advertise("/my_camera/image_raw", 1);
    pose_pub = n.advertise<geometry_msgs::Pose>("/april", 1);
    sub = n.subscribe(ros::names::remap("image"), 1, &Demo::image_callback,this);
  }

  // changing the tag family
  void setTagCodes(string s) {
    if (s=="16h5") {
      m_tagCodes = AprilTags::tagCodes16h5;
    } else if (s=="25h7") {
      m_tagCodes = AprilTags::tagCodes25h7;
    } else if (s=="25h9") {
      m_tagCodes = AprilTags::tagCodes25h9;
    } else if (s=="36h9") {
      m_tagCodes = AprilTags::tagCodes36h9;
    } else if (s=="36h11") {
      m_tagCodes = AprilTags::tagCodes36h11;
    } else {
      cout << "Invalid tag family specified" << endl;
      exit(1);
    }
  }

  // parse command line options to change default behavior
  void parseOptions(int argc, char* argv[]) {
    int c;
    while ((c = getopt(argc, argv, ":h?adC:F:H:S:W:E:G:B:")) != -1) {
      // Each option character has to be in the string in getopt();
      // the first colon changes the error character from '?' to ':';
      // a colon after an option means that there is an extra
      // parameter to this option; 'W' is a reserved character
      switch (c) {
      case 'h':
      case '?':
        cout << intro;
        cout << usage;
        exit(0);
        break;
      case 'a':
        m_arduino = true;
        break;
      case 'd':
        m_draw = false;
        break;
      case 'C':
        setTagCodes(optarg);
        break;
      case 'F':
        m_fx = atof(optarg);
        m_fy = m_fx;
        break;
      case 'H':
        m_height = atoi(optarg);
        m_py = m_height/2;
         break;
      case 'S':
        m_tagSize = atof(optarg);
        break;
      case 'W':
        m_width = atoi(optarg);
        m_px = m_width/2;
        break;
      case 'E':
#ifndef EXPOSURE_CONTROL
        cout << "Error: Exposure option (-E) not available" << endl;
        exit(1);
#endif
        m_exposure = atoi(optarg);
        break;
      case 'G':
#ifndef EXPOSURE_CONTROL
        cout << "Error: Gain option (-G) not available" << endl;
        exit(1);
#endif
        m_gain = atoi(optarg);
        break;
      case 'B':
#ifndef EXPOSURE_CONTROL
        cout << "Error: Brightness option (-B) not available" << endl;
        exit(1);
#endif
        m_brightness = atoi(optarg);
        break;
      case ':': // unknown option, from getopt
        cout << intro;
        cout << usage;
        exit(1);
        break;
      }
    }

    if (argc == optind + 1) {
      m_deviceId = atoi(argv[optind]);
    }
  }

  void setup() {
    m_tagDetector = new AprilTags::TagDetector(m_tagCodes);



    // prepare window for drawing the camera images
    if (m_draw) {
      cv::namedWindow(window_name, 1);
    }

    if(ros::names::remap("image") != "image")
    {
      ROS_INFO("image has been mapped, assuming point grey");
      ros::spin();
      exit(0);
      return;
    }



#ifdef EXPOSURE_CONTROL
    // manually setting camera exposure settings; OpenCV/v4l1 doesn't
    // support exposure control; so here we manually use v4l2 before
    // opening the device via OpenCV; confirmed to work with Logitech
    // C270; try exposure=20, gain=100, brightness=150

    string video_str = "/dev/video0";
    video_str[10] = '0' + m_deviceId;
    int device = v4l2_open(video_str.c_str(), O_RDWR | O_NONBLOCK);

    if (m_exposure >= 0) {
      // not sure why, but v4l2_set_control() does not work for
      // V4L2_CID_EXPOSURE_AUTO...
      struct v4l2_control c;
      c.id = V4L2_CID_EXPOSURE_AUTO;
      c.value = 1; // 1=manual, 3=auto; V4L2_EXPOSURE_AUTO fails...
      if (v4l2_ioctl(device, VIDIOC_S_CTRL, &c) != 0) {
        cout << "Failed to set... " << strerror(errno) << endl;
      }
      cout << "exposure: " << m_exposure << endl;
      v4l2_set_control(device, V4L2_CID_EXPOSURE_ABSOLUTE, m_exposure*6);
    }
    if (m_gain >= 0) {
      cout << "gain: " << m_gain << endl;
      v4l2_set_control(device, V4L2_CID_GAIN, m_gain*256);
    }
    if (m_brightness >= 0) {
      cout << "brightness: " << m_brightness << endl;
      v4l2_set_control(device, V4L2_CID_BRIGHTNESS, m_brightness*256);
    }
    v4l2_close(device);
#endif 

    // find and open a USB camera (built in laptop camera, web cam etc)
    m_cap = cv::VideoCapture(m_deviceId);
        if(!m_cap.isOpened()) {
      cerr << "ERROR: Can't find video device " << m_deviceId << "\n";
      exit(1);
    }
    m_cap.set(CV_CAP_PROP_FRAME_WIDTH, m_width);
    m_cap.set(CV_CAP_PROP_FRAME_HEIGHT, m_height);
    cout << "Camera successfully opened (ignore error messages above...)" << endl;
    cout << "Actual resolution: "
         << m_cap.get(CV_CAP_PROP_FRAME_WIDTH) << "x"
         << m_cap.get(CV_CAP_PROP_FRAME_HEIGHT) << endl;



  }

  void print_detection(AprilTags::TagDetection& detection) const {
  
    if(detection.id != 0)
      return;
  
    cout << "  Id: " << detection.id
         << " (Hamming: " << detection.hammingDistance << ")";

    // recovering the relative pose of a tag:

    // NOTE: for this to be accurate, it is necessary to use the
    // actual camera parameters here as well as the actual tag size
    // (m_fx, m_fy, m_px, m_py, m_tagSize)

    Eigen::Vector3d cameraTranslation; //in camera frame
    Eigen::Matrix3d cameraRotation;    //in camera frame
    detection.getRelativeTranslationRotation(m_tagSize, m_fx, m_fy, m_px, m_py,
                                             cameraTranslation, cameraRotation);
    
    //rotate into body frame
    Eigen::Matrix3d cToM; //camera orientation to body orientation
    cToM <<
      1, 0, 0,  //right of camera points to the right
      0,-1, 0, //Bottom of camera points backward
      0, 0,-1; //Front of camera faces down
    Eigen::Vector3d copterTranslation = cToM*cameraTranslation;
    

    cout << "  distance=" << cameraTranslation.norm()*39.37007874 //convert to inches

         << "\", x=" << copterTranslation(0)
         << ", y=" << copterTranslation(1)

         << ", z=" << copterTranslation(2)
         //<< ", yaw=" << -yaw
         //<< ", pitch=" << pitch
         //<< ", roll=" << roll
         << endl;

    //publish ros
    tf::Matrix3x3 mat;
    mat[0][0] = cToM(0,0); mat[0][1] = cToM(0,1); mat[0][2] = cToM(0,2);
    mat[1][0] = cToM(1,0); mat[1][1] = cToM(1,1); mat[1][2] = cToM(1,2);
    mat[2][0] = cToM(2,0); mat[2][1] = cToM(2,1); mat[2][2] = cToM(2,2);
    tf::Quaternion q;
    mat.getRotation(q);

    //TODO: get frames and such right
    geometry_msgs::Pose to_publish;
    to_publish.position.x = copterTranslation(0);
    to_publish.position.y = copterTranslation(1);
    to_publish.position.z = copterTranslation(2);
    tf::quaternionTFToMsg(q, to_publish.orientation);

    pose_pub.publish(to_publish);

    // Also note that for SLAM/multi-view application it is better to
    // use reprojection error of corner points, because the noise in
    // this relative pose is very non-Gaussian; see iSAM source code
    // for suitable factors.
  }




  void image_callback(const sensor_msgs::ImageConstPtr& picture)
  {
    cv::Mat image;
    cv::Mat image_gray;
    cv_bridge::CvImageConstPtr bridge = cv_bridge::toCvShare(picture,"bgr8");
    cv_bridge::CvImage thresholdPublisher;
    (bridge->image).copyTo(image);
    
      // detect April tags (requires a gray scale image)
      cv::cvtColor(image, image_gray, CV_BGR2GRAY);
      vector<AprilTags::TagDetection> detections = m_tagDetector->extractTags(image_gray);

      // print out each detection
      cout << detections.size() << " tags detected:" << endl;
      for (int i=0; i<detections.size(); i++) {
        print_detection(detections[i]);
      }

      // show the current image including any detections
      if (m_draw) {
        for (int i=0; i<detections.size(); i++) {
          // also highlight in the image
          if(detections[i].id == 0)
            detections[i].draw(image);
        }
        imshow(window_name, image); // OpenCV call
        
        //publish to ros
        cv_bridge::CvImage ros_bridge;
        sensor_msgs::Image ros_image;
        ros_bridge.image = image;
        ros_bridge.encoding = "bgr8";
        ros_bridge.toImageMsg(ros_image);        
        debug_pub.publish(ros_image);
      }

      // print out the frame rate at which image frames are being processed
      static int frame = 0;
      static double last_t = tic();
      frame++;
      if (frame % 10 == 0) {
        double t = tic();
        cout << "  " << 10./(t-last_t) << " fps" << endl;
        last_t = t;
      }

      // exit if any key is pressed
      if (cv::waitKey(1) >= 0) exit(0);    
  }





  // The processing loop where images are retrieved, tags detected,
  // and information about detections generated
  void loop() {

    cv::Mat image;
    cv::Mat image_gray;

    int frame = 0;
    double last_t = tic();
    while (true) {

      // capture frame
      m_cap >> image;

      // alternative way is to grab, then retrieve; allows for
      // multiple grab when processing below frame rate - v4l keeps a
      // number of frames buffered, which can lead to significant lag
      //      m_cap.grab();
      //      m_cap.retrieve(image);

      // detect April tags (requires a gray scale image)
      cv::cvtColor(image, image_gray, CV_BGR2GRAY);
      vector<AprilTags::TagDetection> detections = m_tagDetector->extractTags(image_gray);

      // print out each detection
      cout << detections.size() << " tags detected:" << endl;
      for (int i=0; i<detections.size(); i++) {
        print_detection(detections[i]);
      }

      // show the current image including any detections
      if (m_draw) {
        for (int i=0; i<detections.size(); i++) {
          // also highlight in the image
          if(detections[i].id == 0)
            detections[i].draw(image);
        }
        imshow(window_name, image); // OpenCV call
        
        //publish to ros
        cv_bridge::CvImage ros_bridge;
        sensor_msgs::Image ros_image;
        ros_bridge.image = image;
        ros_bridge.encoding = "bgr8";
        ros_bridge.toImageMsg(ros_image);        
        debug_pub.publish(ros_image);
      }

      // print out the frame rate at which image frames are being processed
      frame++;
      if (frame % 10 == 0) {
        double t = tic();
        cout << "  " << 10./(t-last_t) << " fps" << endl;
        last_t = t;
      }

      // exit if any key is pressed
      if (cv::waitKey(1) >= 0) break;
    }
  }

}; // Demo


// here is were everything begins
int main(int argc, char* argv[]) {

  ros::init(argc, argv, "april_finder");

  Demo demo;

  // process command line options
  demo.parseOptions(argc, argv);

  // setup image source, window for drawing, serial port...
  demo.setup();

  // the actual processing loop where tags are detected and visualized
  demo.loop();

  return 0;
}
