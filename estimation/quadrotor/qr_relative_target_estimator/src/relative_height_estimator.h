/**
 * Definitions of the RelativeHeightEstimator class.
 *
 * Author: K.Ling
 * Dec 2013
 *
 * WAVELab, University of Waterloo
 */

#ifndef QR_RELATIVE_TARGET_ESTIMATOR_SRC_RELATIVE_HEIGHT_ESTIMATOR_H
#define QR_RELATIVE_TARGET_ESTIMATOR_SRC_RELATIVE_HEIGHT_ESTIMATOR_H

#include <wave_estimation/kalman_filter_base.h>


/**
 * Kalman filter to estimate the Pelican quadrotor altitude.
 *
 * This filter fuses the barometer rate and height estimates from the LLP that
 * have already gone through some sort of data fusion with the sonar
 * measurements.
 *
 * States: [z, z_vel, bias_barom_to_sonar, bias_sonar_to_tag]
 *
 * Barometer bias is estimated relative to the sonar measurement.
 */
class RelativeHeightEstimator : public KalmanFilterBase {
  public:
    /**
     * Constructor. Initializes measurement and covariance matrices.
     */
    RelativeHeightEstimator();

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
     * Perform a Kalman filter measurement update with an april tag height
     * measurement.
     *
     * @param[in] z_tag Measured april tag height.
     */
    void tagMeasurementUpdate(double z_tag);

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
    void getStateEstimate(double& z, double& z_vel, double& barom_to_sonar_bias, double& sonar_to_tag_bias);
  
  private:
    // Measurement matrix for sonar only measurement.
    Eigen::MatrixXd sonar_matr_C_;
    // Measurement noise covariance matrix for sonar only measurement.
    Eigen::MatrixXd sonar_meas_cov_;

    // Measurement matrix for barometer only measurement.
    Eigen::MatrixXd barom_matr_C_;
    // Measurement noise covariance matrix for barometer only measurement.
    Eigen::MatrixXd barom_meas_cov_;

    // Measurement matrix for tag only measurement.
    Eigen::MatrixXd tag_matr_C_;
    // Measurement noise covariance matrix for tag only measurement.
    Eigen::MatrixXd tag_meas_cov_;

    // Process noise covariance matrix for prediction step.
    Eigen::MatrixXd pred_cov_;
};

#endif // QR_RELATIVE_TARGET_ESTIMATOR_SRC_RELATIVE_HEIGHT_ESTIMATOR_H

