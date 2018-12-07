/**
 * Definitions of the TargetPositionEstimator class.
 *
 * Author: K.Ling
 * Dec 2013
 *
 * WAVELab, University of Waterloo
 */


#ifndef QR_RELATIVE_TARGET_ESTIMATOR_SRC_TARGET_POSITION_ESTIMATOR_H
#define QR_RELATIVE_TARGET_ESTIMATOR_SRC_TARGET_POSITION_ESTIMATOR_H

#include <wave_estimation/kalman_filter_base.h>

/**
 * Kalman filter to estimate a visual target (tag) in the body-fixed-inertial
 * (BFI) frame of the quadrotor. The BFI is defined as a coordinate frame fixed
 * to the CoG of the quadrotor that is rotated in yaw to be aligned with the
 * front of the quadrotor. The BFI is not rotated in roll or pitch from the
 * inertial frame.
 *
 * The estimation is performed using measurements of the relative position
 * of the april tag fused with the predicted motion of the quadrotor based on
 * the collective thrust and current attitudes.
 *
 * The estimates in the BFI are rotated as the yaw of the quadrotor changes.
 *
 * Estimated states: [relative tag x-position, relative tag y-position,
 *    relative tag x-velocity, relative tag y-velocity]
 *
 * The KalmanFilterBase class is extended in order to provide the underlying
 * Kalman filter functions.
 */
class TargetPositionEstimator : public KalmanFilterBase {
  public:
    /**
     * Constructor. Initializes measurement and covariance matrices.
     */
    TargetPositionEstimator();

    /**
     * Perform a Kalman filter measurement update with only a visual tag
     * measurement.
     *
     * @param[in] tag_x Measured x position of the tag in the BFI.
     * @param[in] tag_y Measured y position of the tag in the BFI.
     */
    void tagMeasurementUpdate(double tag_x, double tag_y);

    /**
     * Performs the prediction update for the Kalman filter. The predictive
     * model assumes that the tag accelerates slowly compared to the quadrotor,
     * so the quadrotor's acceleration is the relative acceleration.
     *
     * @param[in] f_thrust Collective thrust in newtons.
     * @param[in] roll Quadrotor roll angle in radians.
     * @param[in] pitch Quadrotor pitch angle in radians.
     * @param[in] yaw_rate Measured quadrotor yaw rate in radians/s.
     * @param[in] dt Time since last Kalman filter update cycle.
     */
    void predictionUpdate(double qr_acc_x, double qr_acc_y, double yaw_rate,
          double dt);

    /**
     * Get the estimated states.
     *
     * @param[out] z Estimated altitude.
     * @param[out] z_vel Estimated climb rate.
     */
    void getStateEstimate(double &rel_x, double &rel_y, double &rel_v_x, 
          double &rel_v_y);

    /**
     * Reinitializes the tag position estimate.
     *
     * @param[in] tag_x The tag's x-position to reinitialize.
     * @param[in] tag_y The tag's y-position to reinitialize.
     */
    void reinitializeTagPosition(double tag_x, double tag_y);
  
  private:
    Eigen::MatrixXd tag_matr_C;
    Eigen::MatrixXd tag_meas_cov;

    Eigen::MatrixXd pred_matr_A;
    Eigen::MatrixXd pred_cov;
};


#endif // QR_RELATIVE_TARGET_ESTIMATOR_SRC_TARGET_POSITION_ESTIMATOR_H

