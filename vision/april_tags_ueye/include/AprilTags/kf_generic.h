/**
 * A collection of functions and structs for generic linear Kalman Filters.
 *
 * Author: K.Ling
 * Oct 2013
 */

#ifndef __KF_GENERIC_H__
#define __KF_GENERIC_H__

#include <Eigen/Core>
#include <Eigen/LU>

using namespace Eigen;

/**
 * Struct holding the current state of a Kalman Filter. The state includes the
 * actual and predicted mean and covariance matrices.
 *
 * This struct is for a generic Kalman filter. The dimensions should be
 * initialized appropriately.
 */
struct KFContext
{
  VectorXd mean;
  VectorXd pred_mean;
  MatrixXd sigma;
  MatrixXd pred_sigma;
  bool meas_update_last;
};

void kf_measurement_update(KFContext& context, MatrixXd matr_C,
      VectorXd meas_vector, MatrixXd meas_cov);

void kf_prediction_update(KFContext& context, MatrixXd matr_A, VectorXd vect_Bu,
      MatrixXd pred_cov);


#endif //__KF_GENERIC_H__

