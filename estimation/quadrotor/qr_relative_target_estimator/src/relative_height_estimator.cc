/**
 * Source of the RelativeHeightEstimator class.
 *
 * Author: K.Ling
 * Dec 2013
 *
 * WAVELab, University of Waterloo
 */

#include "relative_height_estimator.h"
#include <math.h>

#include <ros/ros.h>

RelativeHeightEstimator::RelativeHeightEstimator() : KalmanFilterBase(4 /*Number of states*/) {
  // for y = [sonar_height]
  sonar_matr_C_ = Eigen::MatrixXd(1, 4);
  sonar_matr_C_ << 1, 0, 0, 1;
  sonar_meas_cov_ = Eigen::MatrixXd(1,1);
  sonar_meas_cov_ <<  0.0025;

  // for y = [barom, climb_rate]
  barom_matr_C_ = Eigen::MatrixXd(2,4);
  barom_matr_C_ << 1, 0, 1, 1,
                  0, 1, 0, 0;
  barom_meas_cov_ = Eigen::MatrixXd(2,2);
  barom_meas_cov_ <<  0.5, 0,
                     0, 0.3;

  // for y = [tag_height]
  tag_matr_C_ = Eigen::MatrixXd(1,4);
  tag_matr_C_ << 1, 0, 0, 0;

  tag_meas_cov_ = Eigen::MatrixXd(1,1);
  tag_meas_cov_ <<  0.5;


  pred_cov_ = Eigen::MatrixXd(4,4);
  pred_cov_ << 0.2, 0, 0, 0,
               0, 0.1, 0, 0,
               0, 0, 0.2, 0,
               0, 0, 0, 0.2;
}


void RelativeHeightEstimator::sonarMeasurementUpdate(double z_sonar) {
  Eigen::VectorXd vect_y(1);
  vect_y << z_sonar;

  Eigen::VectorXd predicted_measurement = sonar_matr_C_ * getMeanStates();

  performMeasurementUpdate(sonar_matr_C_, vect_y, predicted_measurement,
      sonar_meas_cov_);
}


void RelativeHeightEstimator::baromMeasurementUpdate(double z_barom,
      double z_vel) {
  Eigen::VectorXd vect_y(2);
  vect_y << z_barom, z_vel;

  Eigen::VectorXd predicted_measurement = barom_matr_C_ * getMeanStates();

  performMeasurementUpdate(barom_matr_C_, vect_y, predicted_measurement,
      barom_meas_cov_);
}


void RelativeHeightEstimator::tagMeasurementUpdate(double z_tag) {
  Eigen::VectorXd vect_y(1);
  vect_y << z_tag;

  Eigen::VectorXd predicted_measurement = tag_matr_C_ * getMeanStates();

  performMeasurementUpdate(tag_matr_C_, vect_y, predicted_measurement,
      tag_meas_cov_);
}


void RelativeHeightEstimator::predictionUpdate(double f_thrust, double roll,
      double pitch, double qr_mass, double dt) {
  double pred_z_acc;
  if (5 > f_thrust) {
    pred_z_acc = f_thrust*cos(roll)*cos(pitch)/qr_mass - 9.81;
  } else {
    // So it doesn't think it's falling through the ground before takeoff.
    pred_z_acc = 0;
  }

  Eigen::Vector4d vect_Bu(0, pred_z_acc * dt, 0, 0);

  Eigen::MatrixXd pred_matr_A(4,4);
  pred_matr_A << 1, dt, 0, 0,
                 0, 1, 0, 0,
                 0, 0, 1, 0,
                 0, 0, 0, 1;

  Eigen::VectorXd pred_state = pred_matr_A * getMeanStates() + vect_Bu;

  // Don't let the prediction fall through the floor. CoG is about 0.3m above
  // the floor when the QR is landed.
  pred_state(0) = fmax(0.3, pred_state(0));
  if (0.3 == pred_state(0)) { 
    pred_state(1) = 0;
  }

  performPredictionUpdate(pred_matr_A, pred_state, pred_cov_);
}


void RelativeHeightEstimator::getStateEstimate(double& z, double& z_vel,
    double& barom_to_sonar_bias, double& sonar_to_tag_bias) {
  Eigen::VectorXd mean = getMeanStates();
  z = mean(0);
  z_vel = mean(1);
  barom_to_sonar_bias = mean(2);
  sonar_to_tag_bias = mean(3);
}


