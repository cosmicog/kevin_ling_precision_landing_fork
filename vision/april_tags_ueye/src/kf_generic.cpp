#include "kf_generic.h"

using namespace Eigen;


/**
 * Update a specified Kalman filter with a specified measurement vector.
 *
 * @context - Context struct for the Kalman Filter to update
 * @matr_C - The state space 'C' matrix to determine how meas_vector affects
 *    the system
 * @meas_vector - Vector of measured variables to update the filter with
 * @meas_cov - Covariance matrix for the measurements
 */
void kf_measurement_update(KFContext& context, MatrixXd matr_C,
      VectorXd meas_vector, MatrixXd meas_cov)
{
  // TODO: Add input matrix/vector size consistency checks

  VectorXd innovation(matr_C.rows());
  innovation = meas_vector - matr_C * context.pred_mean;

  MatrixXd temp(matr_C.rows(), matr_C.rows());
  temp = matr_C * context.pred_sigma * matr_C.transpose() + meas_cov;

  MatrixXd matr_K(matr_C.cols(), matr_C.rows());
  matr_K = context.pred_sigma * matr_C.transpose() * temp.inverse();

  context.mean = context.pred_mean + matr_K*(innovation);
  context.sigma = ( MatrixXd::Identity(context.sigma.rows(),context.sigma.rows())
        - matr_K*matr_C ) * context.pred_sigma;

  context.meas_update_last = true;
}


/**
 * Update a specified Kalman filter with a prediction step. The effect of the
 * control inputs must be calculated in advanced and passed in as vect_Bu which
 * will be added after the state is multiplied by the state transition matrix,
 * matr_A.
 *
 * @context - Context struct for the Kalman Filter to update
 * @matr_A - The state space 'A' matrix; the state transition matrix
 * @vect_Bu - The pre-computed effect of the control inputs on the predicted
 *    state
 * @pred_cov - The prediction covariance matrix
 */
void kf_prediction_update(KFContext& context, MatrixXd matr_A, VectorXd vect_Bu,
      MatrixXd pred_cov)
{
  // TODO: Add input dimension consitency checks

  VectorXd curr_mean; // the most recently updated mean
  if (context.meas_update_last)
  {
    curr_mean = VectorXd(context.mean);
    context.pred_sigma = matr_A*context.sigma*matr_A.transpose() + pred_cov;
  }
  else
  {
    curr_mean = VectorXd(context.pred_mean);
    context.pred_sigma = matr_A*context.pred_sigma*matr_A.transpose() + pred_cov;
  }

  // X[t] = AX[t-1] + Bu;
  context.pred_mean = matr_A * curr_mean;
  context.pred_mean += vect_Bu;

  context.meas_update_last = false;
}


