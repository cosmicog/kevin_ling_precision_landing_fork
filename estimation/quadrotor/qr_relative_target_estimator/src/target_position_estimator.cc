#include "target_position_estimator.h"

// TODO: See if the inertial velocities of boat and QR should be used for
// prediction updates.

TargetPositionEstimator::TargetPositionEstimator() : KalmanFilterBase(4 /*number of states*/)
{
  tag_matr_C = Eigen::MatrixXd(2,4);
  tag_matr_C << 1, 0, 0, 0,
                0, 1, 0, 0;
  tag_meas_cov = Eigen::MatrixXd(2,2);
  tag_meas_cov << 0.05, 0,
                  0, 0.05;

  pred_cov = Eigen::MatrixXd(4,4);
  pred_cov << 2.5,   0, 0, 0,
                0, 2.5, 0, 0,
                0,   0, 65, 0,
                0,   0, 0, 65;
}


void TargetPositionEstimator::tagMeasurementUpdate(double tag_x, double tag_y)
{
  Eigen::VectorXd matr_y(2);
  matr_y << tag_x, tag_y;

  Eigen::VectorXd predicted_measurement = tag_matr_C * getMeanStates();

  performMeasurementUpdate(tag_matr_C, matr_y, predicted_measurement,
      tag_meas_cov);
}


void TargetPositionEstimator::predictionUpdate(double qr_acc_x, double qr_acc_y,
      double yaw_rate, double dt)
{
  Eigen::Vector4d vect_Bu(0, 0, qr_acc_x*dt, qr_acc_y*dt);


  double delta_yaw = yaw_rate * dt;

  double c_yaw_r = cos(delta_yaw);
  double s_yaw_r = sin(delta_yaw);

  // If the vehicle is yaw-ing, then rotate the current estimates. Need to
  // account for coriolis for the velocity.
  
  // TODO: Double check rotations. Should probably use the full rotation matrix.
  Eigen::MatrixXd matr_A(4, 4);
  matr_A << c_yaw_r,   s_yaw_r,                  dt,                    0,
            -s_yaw_r,  c_yaw_r,                   0,                   dt,
                  0,        0,              c_yaw_r,  s_yaw_r + yaw_rate*2*dt,
                  0,        0, -s_yaw_r - yaw_rate*2*dt,              c_yaw_r;

  Eigen::VectorXd predicted_state = matr_A * getMeanStates() + vect_Bu;

  performPredictionUpdate(matr_A, predicted_state, pred_cov);
}


void TargetPositionEstimator::getStateEstimate(double &rel_x, double &rel_y, double &rel_v_x,
      double &rel_v_y)
{
  Eigen::VectorXd mean = getMeanStates(); 

  rel_x = mean(0);
  rel_y = mean(1);
  rel_v_x = mean(2);
  rel_v_y = mean(3);
}


void TargetPositionEstimator::reinitializeTagPosition(double tag_x, double tag_y)
{
  Eigen::VectorXd old_mean = getMeanStates();

  old_mean(0) = tag_x;
  old_mean(1) = tag_y;

  // TODO: Make these variables
  old_mean(2) = 0;
  old_mean(3) = 0;
  
  // Leave velocities untouched for now... don't really have anything to put
  // for them.
  
  setStateEstimates(old_mean);
}


