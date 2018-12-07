/**
 * Definitions of the KalmanFilterBase class.
 *
 * Author: K.Ling
 * Dec 2013
 *
 * WAVELab, University of Waterloo
 */

#ifndef __KALMAN_FILTER_BASE_HPP__
#define __KALMAN_FILTER_BASE_HPP__

#include <Eigen/Core>
#include <Eigen/LU>


/**
 * An abstract class that implements the basic Kalman Filter equations and
 * encapsulates state estimates and covariances.
 */
class KalmanFilterBase
{

  public:
    /**
     * Constructor for the abstract KalmanFilterBase class. Mean state estimate
     * and covariance matrices will be initialized to dimensions according to
     * the number of states specified.
     *
     * @param[in] num_states Number of states to estimate in the Kalman filter.
     **/
    KalmanFilterBase(int num_states); 

    /**
     * Accessor to get the mean state estimates of the Kalman filter.
     */
    Eigen::VectorXd getMeanStates();

    /**
     * Accessor to get the state estimate covariances of the Kalman filter.
     */
    Eigen::MatrixXd getStateCovariances();

  protected:
    /**
     * Performs the Kalman filter prediction update by updating the mean
     * estimates to predicted_state and calculating the covariances for the
     * new estimates. It is up to the calling method to provide the state
     * prediction based on the mean state estimates.
     *
     * @param[in] matr_A The state space 'A' matrix, or the linearized state
     *    transition matrix for the nonlinear case.
     * @param[in] predicted_state The predicted states, calculated based on the
     *    previous mean state estimates.
     * @param[in] process_noise The 'R' covariance matrix for the system's
     *    process noise.
     **/
    void performPredictionUpdate(Eigen::MatrixXd& matr_A, 
        Eigen::VectorXd& predicted_state, Eigen::MatrixXd& process_noise);

    /**
     * Performs the Kalman filter measurement update. The calling method must
     * calculate the predicted measurement based on the current state estimate
     * first.
     *
     * @param[in] matr_C The state space 'C' matrix, or the linearized
     *    measurement matrix for the nonlinear case.
     * @param[in] measurements Vector of new measurements for the system.
     * @param[in] pred_measurements Vector of predicted measurements.
     * @param[in] measurement_noise The 'Q' covariance matrix for the system's
     *    measurement noise.
     */
    void performMeasurementUpdate(Eigen::MatrixXd& matr_C,
        Eigen::VectorXd& measurements, Eigen::VectorXd& pred_measurements,
        Eigen::MatrixXd& measurement_noise);

    /**
     * Set Kalman filter state estimates. This method should be used to
     * reinitialize the state estimate.
     *
     * @param new_state New mean estimates for the Kalman filter.
     */
    void setStateEstimates(Eigen::VectorXd& new_state);

  private:

    /**
     * The mean estimate of the states.
     */
    Eigen::VectorXd _mean;

    /**
     * Covariances for the current mean estimates.
     */
    Eigen::MatrixXd _sigma;
};

#endif // __KALMAN_FILTER_BASE_HPP__

