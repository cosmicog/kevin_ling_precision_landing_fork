#include <vector>
#include <cmath>

#include <ros/ros.h>

#include <asctec_hl_comm/mav_status.h>
#include <wave_msgs/QRControl.h>
#include <wave_msgs/QRStateEstimate.h>
#include <wave_pelican/pelican_utils.h>
#include <wave_utils/wave_dsp.h>
#include <wave_utils/wave_math.h>
#include <qr_outerloop_pid/SetTranslationControllerStatus.h>

#include <qr_apriltag_follower/apriltag_follower_node.h>

// Assign the default values for the static members of the class
const double AprilTagFollower::default_search_time_threshold_ = 0.1f;
const double AprilTagFollower::default_acquired_time_threshold_ = 3.0f;
const double AprilTagFollower::default_lost_time_threshold_ = 1.0f;
const double AprilTagFollower::UPDATE_RATE = 50.0f;
const double AprilTagFollower::default_follow_x_ = 0.0;
const double AprilTagFollower::default_follow_y_ = 0.0;
const double AprilTagFollower::default_follow_z_ = 3.0;

using namespace std;

AprilTagFollower::AprilTagFollower(ros::NodeHandle nh) 
{  
  // Set up ROS topics
  command_publisher_ = nh.advertise<wave_msgs::QRControl>("wave/qr_command", 1);

  // Subscribe to the Pelican status and relative estimator topics
  ROS_INFO("Subscribing to /fcu/status topic...");
  flight_mode_subscriber_ = nh.subscribe("fcu/status", 1,
      &AprilTagFollower::flightModeSubscriberCallback, this);
  ROS_INFO("Subscribing to /wave/qr_relative_estimate topic...");
  state_estimate_subscriber_ = nh.subscribe("wave/qr_relative_estimate", 1,
      &AprilTagFollower::stateEstimateSubscriberCallback, this);
  
  // Wait until both topics are available
  while( ros::ok() && ( !flight_mode_subscriber_.getNumPublishers() || !state_estimate_subscriber_.getNumPublishers() ) )
  {
    ros::Duration(0.5).sleep();
    ROS_INFO_THROTTLE(5, "Waiting for topics...");
  }
  ROS_INFO("Connected to topics.");
  ROS_INFO(" ");
  
  // Connect to the controller service and wait until it is available
  ROS_INFO("Connecting to wave/translation_controller_status service...");    
  control_service_ = nh.serviceClient<qr_outerloop_pid::SetTranslationControllerStatus>("wave/translation_controller_status");
  control_service_.waitForExistence();
  ROS_INFO("Connected to service.");
  ROS_INFO(" ");

  // Read the time thresholds from the parameter server
  double search_time_threshold, acquired_time_threshold, lost_time_threshold;
  nh.param<double>("wave/search_time_threshold", search_time_threshold, default_search_time_threshold_);
  nh.param<double>("wave/acquired_time_threshold", acquired_time_threshold, default_acquired_time_threshold_);
  nh.param<double>("wave/lost_time_threshold", lost_time_threshold, default_lost_time_threshold_);
  search_time_threshold_ = ros::Duration(search_time_threshold);
  acquired_time_threshold_ = ros::Duration(acquired_time_threshold);
  lost_time_threshold_ = ros::Duration(lost_time_threshold);  

  // Read the relative following positions
  nh.param<double>("wave/follow_x", follow_x_, default_follow_x_);
  nh.param<double>("wave/follow_y", follow_y_, default_follow_y_);
  nh.param<double>("wave/follow_z", follow_z_, default_follow_z_);

  // Initialize the state machine
  ROS_INFO("Initialized. Starting state machine...");
  autonomous_mode_ = false;
  transitionToSEARCH();

  // Timer to update the Waypoint Driver
  run_period_ = 1.0 / UPDATE_RATE;
  run_timer_ = nh.createTimer(ros::Duration(run_period_),
                              &AprilTagFollower::run, this);
}


/**
 * Pelican's LL_Status. Used to determine when the autonomous
 * controller is enabled.
 */
void AprilTagFollower::flightModeSubscriberCallback(
    const asctec_hl_comm::mav_status& msg) 
{      
  autonomous_mode_ = PelicanUtils::checkAutonomousMode(msg);
}

// Callback for receiving a state estimate message
void AprilTagFollower::stateEstimateSubscriberCallback(
    const wave_msgs::QRStateEstimate& msg) 
{     
  // Check that the message is valid before capturing it to the class object  
  bool valid = (    isfinite(msg.x)
                &&  isfinite(msg.y)
                &&  isfinite(msg.z)
                &&  isfinite(msg.roll)
                &&  isfinite(msg.pitch)
                &&  isfinite(msg.yaw)
                &&  isfinite(msg.dx)
                &&  isfinite(msg.dy)
                &&  isfinite(msg.dz)
                &&  isfinite(msg.droll)
                &&  isfinite(msg.dpitch)
                &&  isfinite(msg.dyaw)
                &&  isfinite(msg.p)
                &&  isfinite(msg.q)
                &&  isfinite(msg.r) );
  
  // if this message is valid
  if( valid ) 
  {
    // store the message
    estimator_msg_ = msg;
    state_estimate_rx_count_++; 
    
    // if this message corresponds to a measurement update
    if( estimator_msg_.position_measurement_update ) 
    {
      // record the time
      last_update_time_ = estimator_msg_.header.stamp;
    } 
  }
  else
  {
    // State estimate message is invalid, ignore it.
    ROS_WARN("Invalid Estimator Message Detected! Ignoring...");
  }
}

// Run the state machine for the mission controller
void AprilTagFollower::run(const ros::TimerEvent&) 
{   
  // Find the time since the relative estimate was last updated by an
  // AprilTag measurement.
  ros::Duration time_since_measurement_update = ros::Time::now() - last_update_time_;
  
  // Depending on the state of the machine, do the following...
  switch (follower_state_) 
  {    
    //=======================================================
    case SEARCH:
    {
      // If the user has switched to autonomous mode
      if( autonomous_mode_ )
      {
        // We are not ready, system is LOST
        transitionToLOST();        
        break;
      }
      
      // If we've recently received a state estimate with a tag measurement      
      if( time_since_measurement_update < search_time_threshold_ )
      {
        // Attempt to acquire the target for a period of time
        transitionToACQUIRE();
        break;
      }
      
      break;
    }
      
    //=======================================================  
    case ACQUIRE:
    {
      // If the user has switched to autonomous mode
      if( autonomous_mode_ )
      {
        // We are not ready, system is LOST
        transitionToLOST();
        break;
      }
      
      // If we haven't received a state estimate with a tag measurement in the set time
      if( time_since_measurement_update > lost_time_threshold_ )
      {
        // Go back to SEARCH state
        transitionToSEARCH();        
        break;
      }
      
      // Find time that the target has been consistently tracked
      ros::Duration acquiring_time = ros::Time::now() - time_tracked_;
      
      // If greater than the threshold,
      if( acquiring_time > acquired_time_threshold_ )
      {
        // Target is locked, system is ready to follow
        transitionToARMED();
        break;
      }
      
      break;
    }
      
    //=======================================================
    case ARMED:
    {
      // If we haven't received a state estimate with a tag measurement in the set time
      if( time_since_measurement_update > lost_time_threshold_ )
      {
        // Go back to SEARCH state
        transitionToSEARCH();
        break;
      }
      
      // If the user has switched to autonomous mode
      if( autonomous_mode_ )
      {
        // Begin to follow the target in the relative frame
        transitionToFOLLOW();
        break;
      }
      
      break;
    }
      
    //=======================================================
    case FOLLOW:
    {
      // If we haven't received a state estimate with a tag measurement in the set time
      if( time_since_measurement_update > lost_time_threshold_ )
      {
        // System is LOST
        transitionToLOST();
        break;
      }
      
      // If the user switched back to manual mode
      if( !autonomous_mode_ )
      {
        // Transition to ARMED, ready to follow
        transitionToARMED();      
        break;
      }     
      
      // If we decide it is clear to land
      if( 0 )
      {
        // Begin the landing sequence
        transitionToLAND();
        break;
      }
      
      // Issue the control command to maintain the relative follow position
      control_msg_ = wave_msgs::QRControl();
      control_msg_.type = wave_msgs::QRControl::CMD_RELATIVE_POSITION;
      control_msg_.x = 0;
      control_msg_.y = 0;
      control_msg_.z = follow_z_;
      control_msg_.yaw = 0.0;
      control_msg_.header.stamp = ros::Time::now();
      command_publisher_.publish(control_msg_);
      
      break; 
    }
      
    //=======================================================      
    case LAND:
    {
      // If we haven't received a state estimate with a tag measurement in the set time
      if( time_since_measurement_update > lost_time_threshold_ )
      {
        // System is LOST
        transitionToLOST();
        break;
      }
      
      // If the user switched back to manual mode
      if( !autonomous_mode_ )
      {
        // Transition to ARMED, ready to follow
        transitionToARMED();      
        break;
      }
      
      // If landing is successful
      if( 0 )
      {
        // Mission Accomplished, hold power down state
        transitionToMISSION_ACCOMPLISHED();
        break;
      }         
      
      // Issue the control command to maintain the relative follow position, descend      
      control_msg_ = wave_msgs::QRControl();
      control_msg_.type = wave_msgs::QRControl::CMD_RELATIVE_POSITION;
      control_msg_.x = follow_x_;
      control_msg_.y = follow_y_;
      control_msg_.z = follow_z_;
      control_msg_.yaw = 0.0;
      control_msg_.header.stamp = ros::Time::now();
      command_publisher_.publish(control_msg_);
      
      break;  
    }
    
    //=======================================================      
    case MISSION_ACCOMPLISHED:
    {
      // If the user switched back to manual mode
      if( !autonomous_mode_ )
      {
        // Reset back to SEARCH and attempt to find the target again
        transitionToSEARCH();
        break;
      }     
     
      break;
    }
           
    //=======================================================   
    case LOST:
    {
      // We are LOST, or something else has gone wrong...
      // Command to hover in place, human should take over here.
      
      // If the user switched back to manual mode
      if( !autonomous_mode_ )
      {
        // Reset back to SEARCH and attempt to find the target again
        transitionToSEARCH();
        break;
      }
    }    
    /* fallthrough */
    default:
    {
      // Tell the position control to maintain neutral attitude
      disablePositionControl();
      
      break;
    }
  }
}

bool AprilTagFollower::setPositionControl( bool enable )
{
  qr_outerloop_pid::SetTranslationControllerStatus srv;
  srv.request.enable = enable;
  
  // Try to set the status of the position controller to a maximum number of tries  
  bool success = false;
  int failure_count = 0;
  const int max_failures = 5;
  
  while( !success )
  {
    // Call the service
    success = control_service_.call(srv);
    
    // If the call failed,
    if( !success )
    {
      // Count the failures
      failure_count++;
      
      // If max failures reached
      if( failure_count >= max_failures )
      {
        // Give up trying, this is not a good situation since we cannot
        // command neutral attitude if we are LOST.
        ROS_ERROR("Unable to call Position Controller Status service! This is very bad! Giving up.");
        break;
      }
      else
      {        
        ROS_WARN("Unable to call Position Controller Status service! Retrying...");
      }
    }
  }
  
  // Success if the service call succeeded and the result matches the request
  return success && (srv.response.running == enable);
}
  

int main(int argc, char** argv) 
{
  ROS_INFO("########################################################################");   
  ROS_INFO("                   ____ ");
  ROS_INFO("                  /    \\ ");
  ROS_INFO("     ____         \\____/");
  ROS_INFO("    /    \\________//      ______   ");
  ROS_INFO("    \\____/==/      \\_____/     \\\\ ");
  ROS_INFO("            \\______/====/      // __        __  __  __      __  ______ ");
  ROS_INFO("            //          \\_____//  \\ \\  /\\  / / /  \\ \\ \\    / / / ____/  ");
  ROS_INFO("       ____//                      \\ \\/  \\/ / / /\\ \\ \\ \\  / / / /__ ");
  ROS_INFO("      /      \\\\                     \\  /\\  / / /  \\ \\ \\ \\/ / / /____   ");
  ROS_INFO("     /       //                      \\/  \\/ /_/    \\_\\ \\__/ /______/ ");
  ROS_INFO("     \\______//                     LABORATORY");
  ROS_INFO(" ");
  ROS_INFO("########################################################################");
  ROS_INFO(" ");
  ROS_INFO("Starting Quadrotor AprilTag Follower Mission Control");

  ros::init(argc, argv, "qr_apriltag_follower");
  ros::NodeHandle nh;
  ros::NodeHandle nh_("~");

  AprilTagFollower follower(nh);
  
  ros::spin();
}

