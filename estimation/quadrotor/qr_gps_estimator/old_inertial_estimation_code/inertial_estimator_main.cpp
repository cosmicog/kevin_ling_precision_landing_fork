#include <ros/ros.h>
#include <asctec_msgs/CtrlInput.h>
#include <asctec_msgs/ControllerOutput.h>
#include <asctec_msgs/IMUCalcData.h>
#include <Eigen/Core>
#include <Eigen/LU>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <mav_msgs/Height.h>
#include <tf/tf.h>
#include "inertial_estimator_impl.h"

#define _USE_MATH_DEFINES
#include <math.h>


using namespace Eigen;
using namespace std;


#define QR_MASS 1.634f //kg
#define GRAV 9.81f //m/s/s
#define DRAG_COEFF 1.3f // This number was guessed, should do real sys. ID. later

#define DEG_TO_RAD M_PI/180.0f
#define RAD_TO_DEG 180.0f/M_PI

// -45 degrees to rotate from '+' to 'x' configuration
#define PLUS_TO_X_COS_OFFSET 0.7071067f

#define YAW_RATE_LP_FC 5.0 // LPF cutoff freq
#define YAW_RATE_TO_DEG 0.0154


// Quadratic thrust curve parameters for the Pelican. Identified Oct 2013.
#define THRUST_CURVE_A 0.0002881f
#define THRUST_CURVE_B 0.1286345f
#define THRUST_CURVE_C 128.1525107f

struct globalVarStruct
{
	bool new_thrust;
	bool new_imu;
	
	double thrust;
	double angvel;
	double roll;
	double pitch;
	double yaw;
	
	double gps_x;
	double gps_y;
	
	//lpf variables
	double qr_yaw_rate_lp;
	
	
}g_est;

Matrix3d create_euler_rotation(double roll, double pitch, double yaw)
{
  double cos_roll = cos(roll);
  double sin_roll = sin(roll);
  double cos_pitch = cos(pitch);
  double sin_pitch = sin(pitch);
  double cos_yaw = cos(yaw);
  double sin_yaw = sin(yaw);
 
  Matrix3d rot;
  rot(0, 0) = cos_pitch*cos_yaw;
  rot(0, 1) = cos_pitch*sin_yaw;
  rot(0, 2) = -sin_pitch;
  
  rot(1, 0) = sin_roll*sin_pitch*cos_yaw - cos_roll*sin_yaw;
  rot(1, 1) = sin_roll*sin_pitch*sin_yaw + cos_roll*cos_yaw;
  rot(1, 2) = sin_roll*cos_pitch;

  rot(2, 0) = cos_roll*sin_pitch*cos_yaw + sin_roll*sin_yaw;
  rot(2, 1) = cos_roll*sin_pitch*sin_yaw - sin_roll*cos_yaw;
  rot(2, 2) = cos_roll*cos_pitch;

  return rot;
}

void gps_enu_cb(const geometry_msgs::PointStamped msg)
{
  g_est.gps_x = msg.point.x;
  g_est.gps_y = msg.point.y;
}

void ctrl_output_cb(const asctec_msgs::ControllerOutput& msg)
{
  double commanded_thrust = (double) msg.thrust * 4095.0 / 200.0; // Scale to 0..4095

  // Convert to thrust in N

  g_est.thrust = fmax( 0.0, THRUST_CURVE_A*pow(commanded_thrust,2) + 
        THRUST_CURVE_B*commanded_thrust + THRUST_CURVE_C )*GRAV/1000.0;
        
  g_est.new_thrust = true;
}

void imu_meas_cb(const asctec_msgs::IMUCalcData& msg)
{
	// LPF the yaw rate
  static double imu_calc_last_t = -1;
  static double yaw_rate_lp_tau = 1/(2*M_PI*YAW_RATE_LP_FC);
  double curr_t = ros::Time::now().toSec();
  if (imu_calc_last_t == -1)
  {
    imu_calc_last_t = curr_t;
  }
  else
  {
    double dt = curr_t - imu_calc_last_t;
    imu_calc_last_t = curr_t;

    // Negate yaw rate to correct for NED to ENU change.
    double measured_yaw_rate = -msg.angvel_yaw * YAW_RATE_TO_DEG * DEG_TO_RAD;

    double lp_alpha = dt / (yaw_rate_lp_tau + dt);
    g_est.qr_yaw_rate_lp = lp_alpha*measured_yaw_rate + 
          (1-lp_alpha)*g_est.qr_yaw_rate_lp;
  }

  // IMU Calc message has euler angles in millidegrees. Change to radians.
  Vector3d qr_plus_frame_att(
      ((double)msg.angle_roll/1000.0f) * DEG_TO_RAD,
      ((double)msg.angle_nick/1000.0f) * DEG_TO_RAD,
      ((double)msg.angle_yaw/1000.0f) * DEG_TO_RAD);

  // First rotate by 45 degrees in yaw to go from '+' frame to 'x' frame
  Vector3d qr_cross_frame_att = create_euler_rotation(0,0, M_PI/4.0) * 
        qr_plus_frame_att;

  // Now rotate by 180 degrees in roll to go from NED to ENU
  Vector3d qr_cross_frame_enu_att = create_euler_rotation(M_PI,0,0) * 
        qr_cross_frame_att;


 
  g_est.roll = qr_cross_frame_enu_att(0);
  g_est.pitch = qr_cross_frame_enu_att(1);
  g_est.yaw = qr_cross_frame_enu_att(2);

  g_est.new_imu = true;
}

void init_global_variables()
{
	g_est.new_thrust = false;
	g_est.new_imu = false;
	
	g_est.thrust = 0;
	g_est.angvel = 0;
	g_est.roll = 0;
	g_est.pitch = 0;
	g_est.yaw = 0;
	
	g_est.gps_x = 0;
	g_est.gps_y = 0;
	
	//lpf
	
	g_est.qr_yaw_rate_lp =0;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "wave_inertial_estimator");
  ros::NodeHandle nh;
  
  ros::Subscriber pelican_ctrl_output_sub = 
        nh.subscribe("/asctec/CONTROLLER_OUTPUT", 1, ctrl_output_cb);
 
  ros::Subscriber pelican_imu_sub = 
        nh.subscribe("/asctec/IMU_CALCDATA", 1, imu_meas_cb);
        
  ros::Subscriber gps_data_sub =
		nh.subscribe("/wave/gps_enu", 1, gps_enu_cb);
 
  InertialKF inertial_kf = InertialKF();

  ros::Rate loop_rate(25);

  double dt = 1.0/20.0;
  
  init_global_variables();
 
  while (nh.ok())
  {
    
    //insert filter here
    
    if(g_est.new_thrust && g_est.new_imu)
    {
		//perform motioion update
		g_est.new_thrust = false;
		g_est.new_imu = false;
		inertial_kf.predictionUpdate(g_est.thrust, g_est.roll, g_est.pitch, g_est.qr_yaw_rate_lp, QR_MASS, DRAG_COEFF, dt);
	}
    //done;
    
    //print out the resulting motion
    
    double XI = 0; double YI = 0;
    
    inertial_kf.getStateEstimate(XI,YI);
    
    cout<<XI<<","<<YI<<","<<g_est.gps_x << ","<< g_est.gps_y<< ","<< g_est.pitch << "," << g_est.roll << ","<<g_est.yaw<< endl;
    
    loop_rate.sleep();
    ros::spinOnce();
  }
}
