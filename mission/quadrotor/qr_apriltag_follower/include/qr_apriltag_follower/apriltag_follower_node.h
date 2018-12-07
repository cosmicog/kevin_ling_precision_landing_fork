#ifndef QR_APRILTAG_FOLLOWER_INCLUDE_QR_APRILTAG_FOLLOWER_APRILTAG_FOLLOWER_NODE_H
#define QR_APRILTAG_FOLLOWER_INCLUDE_QR_APRILTAG_FOLLOWER_APRILTAG_FOLLOWER_NODE_H

// Set of states for the Mission Control State Machine
enum AprilTagFollowerState 
{
  SEARCH,
  ACQUIRE,
  ARMED,
  FOLLOW,
  LAND,
  MISSION_ACCOMPLISHED,
  LOST
};

class AprilTagFollower 
{
  public:
    /**
     * Constructor.
     *
     * @param[in] nh ROS nodehandle for creating subscribers and publishers.
     *
     */
    AprilTagFollower(ros::NodeHandle nh);
    ~AprilTagFollower() { };

    void stateEstimateSubscriberCallback(const wave_msgs::QRStateEstimate& msg);
    void flightModeSubscriberCallback(const asctec_hl_comm::mav_status& msg);

    /**
     * Timer callback to drive a list of commands.
     */
    void run(const ros::TimerEvent&);

  private:
    // Count of how many QRStateEstimate messages have been received.
    unsigned int state_estimate_rx_count_;

    // True if autonomous mode is on, determined by the quadrotor's flight mode.
    bool autonomous_mode_;
    
    // Time of last measurement update from an AprilTag measurement
    ros::Time last_update_time_;
    
    // Time that the target was first acquired
    ros::Time time_tracked_;
    
    // Minimum time that an AprilTag measurement must be received, to move to ACQURE state
    ros::Duration search_time_threshold_;
    static const double default_search_time_threshold_;
    
    // Minimum time that the target must be tracked to move to ARMED state
    ros::Duration acquired_time_threshold_;
    static const double default_acquired_time_threshold_;
    
    // Maximum time between receiving estimates updated with tag measurement
    ros::Duration lost_time_threshold_;
    static const double default_lost_time_threshold_;
    
    // Timer duration for the run() timer callback.     
    double run_period_;
    static const double UPDATE_RATE;

    ros::Subscriber state_estimate_subscriber_;
    ros::Subscriber flight_mode_subscriber_;

    ros::Publisher command_publisher_;
    
    // Interface to Position Controller Status service
    ros::ServiceClient control_service_;

    ros::Timer run_timer_;

    // The quadrotor will hold the start position on transition to FOLLOW   
    double follow_x_;
    double follow_y_; 
    double follow_z_;
    static const double default_follow_x_;
    static const double default_follow_y_;
    static const double default_follow_z_; 

    // Last issued command message. Useful for checking if the target is
    // reached.
    wave_msgs::QRControl control_msg_;
    
    // Last received estimator message.
    wave_msgs::QRStateEstimate estimator_msg_;
        
    // State Machine State and Transitions
    AprilTagFollowerState follower_state_;
    
    void transitionToSEARCH() 
    {
      ROS_INFO("+++++++++++++++++++");
      ROS_INFO("+ State: {SEARCH} +"); 
      ROS_INFO("+++++++++++++++++++");     
      follower_state_ = SEARCH;
      
      estimator_msg_ = wave_msgs::QRStateEstimate();
      disablePositionControl();
    } 
    void transitionToACQUIRE() 
    {
      ROS_INFO("++++++++++++++++++++");
      ROS_INFO("+ State: {ACQUIRE} +");
      ROS_INFO("++++++++++++++++++++");
      follower_state_ = ACQUIRE;
      
      time_tracked_ = ros::Time::now();
      disablePositionControl();
    } 
    void transitionToARMED() 
    {
      ROS_INFO("++++++++++++++++++");
      ROS_INFO("+ State: {ARMED} +");
      ROS_INFO("++++++++++++++++++");
      follower_state_ = ARMED;
      
      disablePositionControl();
    } 
    void transitionToFOLLOW() 
    {
      ROS_INFO("+++++++++++++++++++");
      ROS_INFO("+ State: {FOLLOW} +");
      ROS_INFO("+++++++++++++++++++");
      follower_state_ = FOLLOW;
      
      follow_x_ = estimator_msg_.x;
      follow_y_ = estimator_msg_.y;
      follow_z_ = estimator_msg_.z;
      ROS_INFO_STREAM("Follow Hold: [ " << follow_x_ << ", " << follow_y_ << ", " << follow_z_ << " ]");
      
      enablePositionControl();
    }
    void transitionToLAND() 
    {
      ROS_INFO("+++++++++++++++++");
      ROS_INFO("+ State: {LAND} +");
      ROS_INFO("+++++++++++++++++");
      follower_state_ = LAND;
      
      enablePositionControl();
    }
    void transitionToMISSION_ACCOMPLISHED() 
    {
      ROS_INFO("+++++++++++++++++++++++++++++++++");
      ROS_INFO("+ State: {MISSION_ACCOMPLISHED} +");
      ROS_INFO("+++++++++++++++++++++++++++++++++");
      follower_state_ = MISSION_ACCOMPLISHED;
      
      enablePositionControl();
    }
    void transitionToLOST() 
    {
      ROS_INFO("+++++++++++++++++");
      ROS_INFO("+ State: {LOST} +");
      ROS_INFO("+++++++++++++++++");
      follower_state_ = LOST;
      
      estimator_msg_ = wave_msgs::QRStateEstimate();
      disablePositionControl();
    }
    
    // Call the Position Control Status service to enable/disable the controller
    bool setPositionControl( bool enable );
    void disablePositionControl() 
    {    
      bool success = setPositionControl(false);
    } 
    void enablePositionControl() 
    {
      bool success = setPositionControl(true);
    }
};


#endif

