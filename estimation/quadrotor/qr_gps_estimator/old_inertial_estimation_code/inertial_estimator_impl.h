
/**
 * Inertial Position Estimator
 *
 * Author: A.Das
 * Dec 2013
 *
 * WAVELab, University of Waterloo
 */

#ifndef __INERTIALESTIMATORIMPL_H__
#define __INERTIALESTIMATORIMPL_H__

#include "KalmanFilterBase.hpp"

class InertialKF : public KalmanFilterBase
{
  public:
    /**
     * Constructor. Initializes measurement and covariance matrices.
     */
    InertialKF();
   
    void gpsMeasurementUpdate(double x_pos,double y_pos,double x_vel,double y_vel);
  
    void magnetometerMeasurementUpdate(double heading);
    
    void fullMeasurementUpdate(double x_pos,double y_pos,double x_vel,double y_vel, double heading);
   
    void predictionUpdate(double f_thrust, double roll, double pitch, double yaw_rate, double qr_mass, double qr_drag, double dt);
    
    void getStateEstimate(double &X, double &Y); //add more states if required
  
  private:
    // Measurement matrix for gps only measurement.
    Eigen::MatrixXd gps_matr_C;
    // Measurement noise covariance matrix for gps only measurement.
    Eigen::MatrixXd gps_meas_cov;

    // Measurement matrix for magnetometer only measurement.
    Eigen::MatrixXd magnet_matr_C;
    // Measurement noise covariance matrix for magnetometer only measurement.
    Eigen::MatrixXd magnet_meas_cov;

    // Measurement matrix for full.
    Eigen::MatrixXd full_matr_C;
    // Measurement noise covariance matrix for full measurements together.
    Eigen::MatrixXd full_meas_cov;

    // Process noise covariance matrix for prediction step.
    Eigen::MatrixXd pred_cov;
};

#endif 

