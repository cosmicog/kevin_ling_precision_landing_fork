/**
 * // TODO: Rewrite this comment block... I think it's almost completely
 * // defunct.
 *
 * The global estimator is so named because it includes several sub estimators
 * which together estimate almost all of the high level states on the vehicle.
 * This node is intended for the quadrotor boat landing project using an AscTec
 * Pelican quadrotor equipped with a PX4Flow.
 *
 * Sub estimators included in this node are:
 *
 * 1. Quadrotor ground speed estimator. Kalman filter that takes in PX4Flow
 *    data and uses attitude estimates from the LLP for the predicitve model.
 *    This does not use magnetometers or GPS.
 *
 *    Estimation is in body-fixed-inertial frame.
 *
 * 2. Boat position and velocity estimator. Kalman filter with constant
 *    velocity or an LPF on boat GPS data.
 *
 *    Estimation is in inertial frame.
 *
 * 3. Relative position and relative velocity estimator. Kalman filter that
 *    takes in April Tag relative position measurements and the outputs of
 *    the two previous estimators. The boat is assumed to have a constant
 *    velocity so the prediction model only takes quadrotor acceleration
 *    into account for the relative acceleration.
 *
 *    Estimation is in the body-fixed-inertial frame.
 *
 * Note: The body-fixed-inertial (BFI) frame is defined as a coordinate frame
 * with its XY plane parallel to the inertial XY plane and its yaw rotated with
 * the quadrotor's true heading such that the front of the quadrotor is always
 * pointing in the positive X direction.
 *
 * Author: Kevin Ling
 * October 2013
 */

#define _USE_MATH_DEFINES
#include <math.h>

#include <ros/ros.h>

#include <Eigen/Core>
#include <Eigen/LU>

#include <px_comm/OpticalFlow.h>

#include <geometry_msgs/PoseStamped.h>
#include <asctec_hl_comm/mav_imu.h>
#include <asctec_hl_comm/MotorSpeed.h>

#include <wave_pelican/pelican_utils.h>
#include <wave_utils/wave_math.h>
#include <wave_utils/wave_dsp.h>
#include <wave_msgs/QRStateEstimate.h>
#include <wave_msgs/QRRelativeHeight.h>

#include "target_position_estimator.h"
#include "height_estimator.h"
#include "relative_height_estimator.h"

#define QR_MASS_KG 1.64

#define IMU_RING_BUF_LEN 20

#define DEBUG_MSGS

// This number was guessed, should do real sys ID later
// Aerodynamic drag coefficient of the quadrotor.
#define DRAG_COEFF 2.0

TargetPositionEstimator tag_pos_kf = TargetPositionEstimator();
TargetPositionEstimator tag_pos_prediction_only_kf = TargetPositionEstimator();
HeightEstimator height_kf = HeightEstimator();
RelativeHeightEstimator relative_height_kf = RelativeHeightEstimator();

// Global variables
struct GlobalEstimator
{
  // QR attitudes from IMUCalc
  double qr_roll_rad[IMU_RING_BUF_LEN]; // Ring buffer
  double qr_pitch_rad[IMU_RING_BUF_LEN];
  double qr_yaw_rate_lp;

  int imu_buffer_index;

  // Tag measurements, rotated into BFI
  double tag_roll;
  double tag_pitch;
  double tag_yaw;
  bool tag_lost;
  ros::Time last_tag_time;

  bool barom_only_height_reached;
  
  // Fused height
  double qr_height;

  double qr_z_barom;
  double z_sonar;

  // From CTRL Input
  double qr_thrust;

  // ROS Pubs/Subs/Msgs
  ros::Publisher rel_pos_kf_pub;
  wave_msgs::QRStateEstimate tag_est_msg;

  // DEBUG ONLY
  ros::Publisher pred_only_pub;

  ros::Publisher tag_window_pub;
  geometry_msgs::PointStamped tag_window_msg;

  ros::Publisher relative_height_pub;

#ifdef DEBUG_MSGS
  // April tag measurement rotated into BFI frame
  geometry_msgs::Pose rotated_tag;
  ros::Publisher rotated_tag_pub;

  // Attitude correction from '+' to 'x' frame
  geometry_msgs::PointStamped corrected_attitudes;
  ros::Publisher corrected_attitudes_pub;

  // Attitudes measured by april tags, assuming target is flat on ground plane
  geometry_msgs::PointStamped tag_attitudes;
  ros::Publisher tag_attitudes_pub;

  double inertial_yaw;
#endif

  geometry_msgs::PointStamped alt_kf_msg;
  ros::Publisher alt_kf_pub;

  bool recent_position_measurement;

  int imu_delay_frames;
} g_est;


void heightPredictionUpdate() {
  static ros::Time last_prediction_time(0);
  ros::Time current_time = ros::Time::now();
  double dt = (current_time - last_prediction_time).toSec();
  last_prediction_time = current_time;
  if (dt > 1) {
    return;
  }

  height_kf.predictionUpdate(g_est.qr_thrust, g_est.qr_roll_rad[g_est.imu_buffer_index],
                             g_est.qr_pitch_rad[g_est.imu_buffer_index], QR_MASS_KG, dt);
  relative_height_kf.predictionUpdate(g_est.qr_thrust, g_est.qr_roll_rad[g_est.imu_buffer_index],
                             g_est.qr_pitch_rad[g_est.imu_buffer_index], QR_MASS_KG, dt);

}

void positionPredictionUpdate() {
  static ros::Time last_prediction_time(0);
  ros::Time current_time = ros::Time::now();
  double dt = (current_time - last_prediction_time).toSec();
  last_prediction_time = current_time;
  if (dt > 1) {
    return;
  }

  double qr_acc_x = 0;
  double qr_acc_y = 0;

  if (g_est.qr_height > 0.15) {
    double c_roll = cos(g_est.qr_roll_rad[g_est.imu_buffer_index]);
    double s_roll = sin(g_est.qr_roll_rad[g_est.imu_buffer_index]);
    double s_pitch = sin(g_est.qr_pitch_rad[g_est.imu_buffer_index]);

    // These will be unused.
    double tag_pos_x = 0;
    double tag_pos_y = 0;

    // For calculating drag.
    double tag_vel_x = 0;
    double tag_vel_y = 0;

    // Relative velocity estimate is better than no estimate.
    tag_pos_kf.getStateEstimate(tag_pos_x, tag_pos_y, tag_vel_x, tag_vel_y);
    
    double tag_force_x_due_to_inputs = -(c_roll*s_pitch) * g_est.qr_thrust;
    double tag_force_x_due_to_drag = -tag_vel_x*DRAG_COEFF;
    double tag_acc_x = (tag_force_x_due_to_inputs + tag_force_x_due_to_drag) /
        QR_MASS_KG;
    
    double tag_acc_y = ( -(-s_roll ) * g_est.qr_thrust -
        tag_vel_y*DRAG_COEFF ) / QR_MASS_KG;


    qr_acc_x = tag_acc_x;
    qr_acc_y = tag_acc_y;
/*
    fprintf(stderr, "-----------------------\n");
    fprintf(stderr, "fqrx: %f, fdx: %f\n", tag_force_x_due_to_inputs, tag_force_x_due_to_drag);
    fprintf(stderr, "croll: %f, spitch: %f\n", c_roll, s_pitch);
    fprintf(stderr, "vx: %f, vy: %f\n", tag_vel_x, tag_vel_y);
    fprintf(stderr, "r: %f, p: %f\n", g_est.qr_roll_rad, g_est.qr_pitch_rad);
    fprintf(stderr, "thrust: %f\n", g_est.qr_thrust);
    fprintf(stderr, "acc-x: %f, acc-y: %f\n", qr_acc_x, qr_acc_y);
    fprintf(stderr, "yaw_rate: %f\n", g_est.qr_yaw_rate_lp);

    static double accumulator = 0;
    accumulator += g_est.qr_yaw_rate_lp * dt;
    fprintf(stderr, "dead_yaw: %f\n", accumulator);
*/
  }

  tag_pos_kf.predictionUpdate(qr_acc_x, qr_acc_y, g_est.qr_yaw_rate_lp, dt);
//  tag_pos_prediction_only_kf.predictionUpdate(qr_acc_x, qr_acc_y,
//      g_est.qr_yaw_rate_lp, dt);
}


// Increase in height between pings that is considered a bad reading. With this
// sonar we haven't seen any false readings be for a lower altitude, so don't
// worry about that.
#define SONAR_SPIKE_THRESHOLD 0.8

#define SONAR_GROUND_THRESHOLD 0.35

void px4_meas_cb(const px_comm::OpticalFlow& msg) {
  static double last_sonar_measurement = 0;

  bool is_new_value = last_sonar_measurement != msg.ground_distance;
  bool is_sonar_spike = (msg.ground_distance - last_sonar_measurement) >
      SONAR_SPIKE_THRESHOLD;

  heightPredictionUpdate();
  
  // Get sonar data
  if(g_est.barom_only_height_reached) {
    return;
  }

  if (is_new_value && !is_sonar_spike) {
    last_sonar_measurement = msg.ground_distance;

    double tilt_correction = cos(g_est.qr_roll_rad[g_est.imu_buffer_index]) * cos(g_est.qr_pitch_rad[g_est.imu_buffer_index]);
    tilt_correction = fmax(tilt_correction, 0.707f);
    g_est.z_sonar = msg.ground_distance * tilt_correction;

    height_kf.sonarMeasurementUpdate(g_est.z_sonar);
    relative_height_kf.sonarMeasurementUpdate(g_est.z_sonar);

  } else if (last_sonar_measurement < SONAR_GROUND_THRESHOLD) {
    // If it's still on the ground, treat it as a new measurement so the barom
    // bias estimate is still correct.
    height_kf.sonarMeasurementUpdate(SONAR_GROUND_THRESHOLD);
    relative_height_kf.sonarMeasurementUpdate(SONAR_GROUND_THRESHOLD);
  }
}

void motorSpeedSubscriberCallback(const asctec_hl_comm::MotorSpeed& msg) {
  // Average four motors for collective thrust
  double collective_command = msg.motor_speed.at(0) + msg.motor_speed.at(1) + 
                              msg.motor_speed.at(2) + msg.motor_speed.at(3);
  collective_command /= 4.0;

  // Normalize motor speed to 0..1
  collective_command /= 200.0;

  // Convert to thrust in N
  PelicanUtils& pelican_utils = PelicanUtils::getInstance();
  g_est.qr_thrust = pelican_utils.thrustCommandToCollectiveForce(collective_command);
}


void imu_meas_cb(const asctec_hl_comm::mav_imu& msg) {
  Eigen::Vector4d qr_orientation(msg.orientation.x, msg.orientation.y,
                                 msg.orientation.z, msg.orientation.w);
  Eigen::Vector3d qr_plus_frame_rpy =
      wave_utils::quaternionToEuler321(qr_orientation, 0.001);

  // First rotate by 45 degrees in yaw to go from '+' frame to 'x' frame
  Eigen::Vector3d qr_cross_frame_rpy = wave_utils::createDCMFromRPY(0, 0, -M_PI/4.0) *
        qr_plus_frame_rpy;

#ifdef DEBUG_MSGS
  g_est.corrected_attitudes.header.stamp = ros::Time::now();
  g_est.corrected_attitudes.point.x = qr_cross_frame_rpy(0);
  g_est.corrected_attitudes.point.y = qr_cross_frame_rpy(1);
  g_est.corrected_attitudes.point.z = 0; // Who cares
  g_est.corrected_attitudes_pub.publish(g_est.corrected_attitudes);
#endif

  g_est.imu_buffer_index = (g_est.imu_buffer_index + 1) % IMU_RING_BUF_LEN;
  g_est.qr_roll_rad[g_est.imu_buffer_index] = qr_cross_frame_rpy(0);
  g_est.qr_pitch_rad[g_est.imu_buffer_index] = qr_cross_frame_rpy(1);

  g_est.qr_z_barom = msg.height;
  double qr_z_climb_rate = msg.differential_height;

  heightPredictionUpdate();
  height_kf.baromMeasurementUpdate(g_est.qr_z_barom, qr_z_climb_rate);
  relative_height_kf.baromMeasurementUpdate(g_est.qr_z_barom, qr_z_climb_rate);
}


// z-axis translation from camera to centre of rotation of the vehicle
#define QR_CAMERA_OFFSET 0.15f

// Receive the April tag detection callback and rotate the data into the body-
// fixed-inertial frame.
void april_tag_cb(const geometry_msgs::PoseStamped& msg) {
  // Transformation from BFI to quad
  
  int old_imu_data_index = (g_est.imu_buffer_index + (IMU_RING_BUF_LEN - g_est.imu_delay_frames)) % IMU_RING_BUF_LEN;

  Eigen::Matrix3d r_bfi2quad = wave_utils::createDCMFromRPY(
        g_est.qr_roll_rad[old_imu_data_index], g_est.qr_pitch_rad[old_imu_data_index], 0).transpose();

  Eigen::Vector3d meas_tag(msg.pose.position.x, msg.pose.position.y,
                           msg.pose.position.z);
  Eigen::Vector3d corrected_tag = r_bfi2quad * meas_tag;

  double tag_x = corrected_tag(0);
  double tag_y = corrected_tag(1);
  double tag_z = corrected_tag(2);

  tf::Quaternion q;
  tf::quaternionMsgToTF(msg.pose.orientation, q);
  Eigen::Vector3d measured_tag_attitude =
      wave_utils::tfQuaternionToEuler321(q, 0.001);
  Eigen::Vector3d corrected_tag_attitude = r_bfi2quad * measured_tag_attitude;

  // TODO: Correct for inertial yaw
  g_est.tag_roll = corrected_tag_attitude(0);
  g_est.tag_pitch = corrected_tag_attitude(1);
  g_est.tag_yaw = corrected_tag_attitude(2);

#ifdef DEBUG_MSGS
  g_est.rotated_tag.position.x = tag_x;
  g_est.rotated_tag.position.y = tag_y;
  g_est.rotated_tag.position.z = tag_z;
  g_est.rotated_tag_pub.publish(g_est.rotated_tag);

  g_est.tag_attitudes.header.stamp = ros::Time::now();
  g_est.tag_attitudes.point.x = wave_utils::radToDeg(g_est.tag_roll);
  g_est.tag_attitudes.point.y = wave_utils::radToDeg(g_est.tag_pitch);
  g_est.tag_attitudes.point.z = wave_utils::radToDeg(g_est.tag_yaw);
  g_est.tag_attitudes_pub.publish(g_est.tag_attitudes);
#endif

  if (g_est.tag_lost) {
    tag_pos_kf.reinitializeTagPosition(-tag_x, -tag_y);
  }

  g_est.tag_lost = false;
  g_est.last_tag_time = ros::Time::now();

  positionPredictionUpdate();
  tag_pos_kf.tagMeasurementUpdate(-tag_x, -tag_y);

  g_est.recent_position_measurement = true;

  heightPredictionUpdate();
  relative_height_kf.tagMeasurementUpdate(-tag_z);
}


void reproject_tag_pos_to_camera_and_publish() {
  // BFI to camera frame
  Eigen::Matrix3d r_bfi2c = wave_utils::createDCMFromRPY(
        g_est.qr_roll_rad[g_est.imu_buffer_index],
        g_est.qr_pitch_rad[g_est.imu_buffer_index],
        0);

  Eigen::Vector3d pred_tag(g_est.tag_est_msg.x, g_est.tag_est_msg.y,
                           g_est.tag_est_msg.z);
  Eigen::Vector3d pred_tag_in_cf = r_bfi2c * pred_tag;

  g_est.tag_window_msg.header.stamp = ros::Time::now();
  g_est.tag_window_msg.point.x = pred_tag_in_cf(0);
  g_est.tag_window_msg.point.y = pred_tag_in_cf(1);
  g_est.tag_window_msg.point.z = pred_tag_in_cf(2);
  g_est.tag_window_pub.publish(g_est.tag_window_msg);
}


void init_global_variables() {
  // QR attitudes from IMUCalc
  g_est.qr_roll_rad[0] = 0;
  g_est.qr_pitch_rad[0] = 0;
  g_est.qr_yaw_rate_lp = 0;

  g_est.imu_buffer_index = 0;

  // Tag measurements, rotated into BFI
  g_est.tag_roll = 0;
  g_est.tag_pitch = 0;
  g_est.tag_yaw = 0;
  g_est.tag_lost = true;
  g_est.last_tag_time = ros::Time::now();;

  g_est.qr_height = 0;

  g_est.qr_z_barom = 0;
  g_est.z_sonar = 0;

  // From CTRL Input
  g_est.qr_thrust = 0;

  g_est.barom_only_height_reached = false;

  g_est.inertial_yaw = 0;

  g_est.recent_position_measurement = false;
}


void publishAndMonitorTimerCallback(const ros::TimerEvent&) {
  if ((ros::Time::now() - g_est.last_tag_time).toSec() > 2.0) {
    // We've been out of tracking too long
    g_est.tag_lost = true;
  }

  positionPredictionUpdate();

  g_est.tag_est_msg.position_measurement_update = g_est.recent_position_measurement;
  g_est.recent_position_measurement = false;

  double x, y, dx, dy;
  tag_pos_kf.getStateEstimate(x, y, dx, dy);
  
  g_est.tag_est_msg.x = x;
  g_est.tag_est_msg.y = y;
  g_est.tag_est_msg.dx = dx;
  g_est.tag_est_msg.dy = dy;

  g_est.tag_est_msg.header.stamp = ros::Time::now();
  g_est.rel_pos_kf_pub.publish(g_est.tag_est_msg);

  g_est.alt_kf_msg.header.stamp = ros::Time::now();
  height_kf.getStateEstimate(g_est.alt_kf_msg.point.x, g_est.alt_kf_msg.point.y,
                             g_est.alt_kf_msg.point.z);
  g_est.alt_kf_pub.publish(g_est.alt_kf_msg);

  g_est.qr_height = g_est.alt_kf_msg.point.x;

  g_est.tag_est_msg.z = g_est.qr_height;
  g_est.tag_est_msg.dz = g_est.alt_kf_msg.point.y;

  g_est.tag_est_msg.roll = g_est.corrected_attitudes.point.x;
  g_est.tag_est_msg.pitch = g_est.corrected_attitudes.point.y;

  if (g_est.qr_height > 3.9 && !g_est.barom_only_height_reached) {
    ROS_INFO("barom only height");
    g_est.barom_only_height_reached = true;

  } else if ((g_est.qr_z_barom < 5.0) && (g_est.z_sonar < 3.7) &&
             g_est.barom_only_height_reached) {

    g_est.barom_only_height_reached = false;
    ROS_INFO("Barom only off");
  }

  if (!g_est.tag_lost) {
    reproject_tag_pos_to_camera_and_publish();
  }
/*
  // Prediction only position estimator
  //
  //
  wave_msgs::QRPosition pred_only_message;
  pred_only_message.is_tracking = false;
  pred_only_message.header.stamp = g_est.tag_est_msg.header.stamp;
  tag_pos_prediction_only_kf.getStateEstimate(
      pred_only_message.pos_x,
      pred_only_message.pos_y,
      pred_only_message.vel_x,
      pred_only_message.vel_y);
  g_est.pred_only_pub.publish(pred_only_message);
*/

  //
  // Publish relative height estimates
  //
  wave_msgs::QRRelativeHeight relative_height_msg;
  relative_height_msg.header.stamp = ros::Time::now();

  relative_height_kf.getStateEstimate(relative_height_msg.z,
                                      relative_height_msg.dz,
                                      relative_height_msg.barom_to_sonar_bias,
                                      relative_height_msg.sonar_to_tag_bias);
  g_est.relative_height_pub.publish(relative_height_msg);
}


int main(int argc, char** argv) {
  ros::init(argc, argv, "qr_relative_target_estimator");
  ros::NodeHandle nh;

  ros::Subscriber ctrl_output_sub =
        nh.subscribe("/fcu/motor_speed", 1, motorSpeedSubscriberCallback);
  ros::Subscriber px4flow_sub =
        nh.subscribe("/px4flow/opt_flow", 1, px4_meas_cb);
  ros::Subscriber pelican_imu_sub =
        nh.subscribe("/fcu/imu_custom", 1, imu_meas_cb);
  ros::Subscriber april_tag_sub =
        nh.subscribe("/wave/tag", 1, april_tag_cb);

  g_est.rel_pos_kf_pub =
        nh.advertise<wave_msgs::QRStateEstimate>("/wave/qr_relative_estimate",
                                                 1);
  g_est.tag_window_pub =
        nh.advertise<geometry_msgs::PointStamped>("/wave/tag_window", 1);

  // DEBUG
  g_est.pred_only_pub =
        nh.advertise<wave_msgs::QRStateEstimate>("/wave/pred_pos_kf", 1);

  g_est.relative_height_pub =
        nh.advertise<wave_msgs::QRRelativeHeight>("/wave/relative_height", 1);

#ifdef DEBUG_MSGS
  g_est.rotated_tag_pub =
        nh.advertise<geometry_msgs::PoseStamped>("/wave/rotated_tag", 1);
//  ros::Subscriber ips_sub = nh.subscribe("/Robot_1/pose", 1, ipsCallback);
g_est.tag_attitudes_pub =
        nh.advertise<geometry_msgs::PointStamped>("/wave/tag_attitudes",1);
  g_est.corrected_attitudes_pub =
        nh.advertise<geometry_msgs::PointStamped>("/wave/corrected_att",1);
#endif

  g_est.alt_kf_pub = nh.advertise<geometry_msgs::PointStamped>("/wave/alt_kf", 1);

  nh.param<int>("wave/imu_delay_frames", g_est.imu_delay_frames, 6);
  ROS_INFO("Relative Estimator: IMU Delay frames set to %d", g_est.imu_delay_frames);

  init_global_variables();

  // Run initialization loop to get filters to stabilize
  ROS_INFO("Initializing estimators for 3 seconds...");
  ros::Rate loop_rate(50);
  ros::Time start = ros::Time::now();
  while(nh.ok() && ((ros::Time::now() - start).toSec() < 3)) {
    loop_rate.sleep();
    ros::spinOnce();
  }
  ROS_INFO("Estimator initialization finished.");

  // Register timers for outputs.
  ros::Timer publish_and_monitor_timer = nh.createTimer(ros::Duration(0.025),
      publishAndMonitorTimerCallback);

  ros::spin();
}

