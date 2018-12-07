/**
 * Definitions of the HeightEstimator class.
 *
 * Author: K.Ling
 * Dec 2013
 *
 * WAVELab, University of Waterloo
 */

#ifndef QR_RELATIVE_TARGET_ESTIMATOR_SRC_HEIGHT_ESTIMATOR_H
#define QR_RELATIVE_TARGET_ESTIMATOR_SRC_HEIGHT_ESTIMATOR_H

#include <wave_estimation/kalman_filter_base.h>


/**
 * Kalman filter to estimate the Pelican quadrotor altitude.
 *
 * This filter fuses the barometer rate and height estimates from the LLP that
 * have already gone through some sort of data fusion with the sonar
 * measurements.
 *
 * States: [z, z_vel, bias_barom]
 *
 * Barometer bias is estimated relative to the sonar measurement.
 */
class HeightEstimator : public KalmanFilterBase {
  public:
    /**
     * Constructor. Initializes measurement and covariance matrices.
     */
    HeightEstimator();

    /**
     * Perform a Kalman filter measurement update with only a new sonar
     * measurement.
     *
     * @param[in] z_sonar Measured sonar height.
     */
    void sonarMeasurementUpdate(double z_sonar);

    /**
     * Perform a Kalman filter measurement update with only a new barometer
     * measurement.
     *
     * @param[in] z_barom Measured height from the barometer.
     * @param[in] z_vel Measured climb rate from the LLP.
     */
    void baromMeasurementUpdate(double z_barom, double z_vel);

    /**
     * Perform a Kalman filter measurement update with both a new sonar and
     * barometer measurement.
     *
     * @param[in] z_sonar Measured sonar height.
     * @param[in] z_barom Measured height from the barometer.
     * @param[in] z_vel Measured climb rate from the LLP.
     */
    void fullMeasurementUpdate(double z_sonar, double z_barom, double z_vel);

    /*
     * Perform the Kalman filter prediction update.
     *
     * @param[in] f_thrust Total thrust in newtons.
     * @param[in] roll Quadrotor roll in radians.
     * @param[in] pitch Quadrotor pitch in radians.
     * @param[in] qr_mass Mass of the quadrotor system.
     * @param[in] dt Time elapsed since last update.
     */
    void predictionUpdate(double f_thrust, double roll, double pitch,
          double qr_mass, double dt);

    /**
     * Get the estimated states.
     *
     * @param[out] z Estimated altitude.
     * @param[out] z_vel Estimated climb rate.
     */
    void getStateEstimate(double &z, double &z_vel, double& barom_bias);
  
  private:
    // Measurement matrix for sonar only measurement.
    Eigen::MatrixXd sonar_matr_C_;
    // Measurement noise covariance matrix for sonar only measurement.
    Eigen::MatrixXd sonar_meas_cov_;

    // Measurement matrix for barometer only measurement.
    Eigen::MatrixXd barom_matr_C_;
    // Measurement noise covariance matrix for barometer only measurement.
    Eigen::MatrixXd barom_meas_cov_;

    // Measurement matrix for sonar and barometer measurements together.
    Eigen::MatrixXd full_matr_C_;
    // Measurement noise covariance matrix for sonar and barometer measurements together.
    Eigen::MatrixXd full_meas_cov_;

    // Process noise covariance matrix for prediction step.
    Eigen::MatrixXd pred_cov_;
};

#endif // QR_RELATIVE_TARGET_ESTIMATOR_SRC_HEIGHT_ESTIMATOR_H

