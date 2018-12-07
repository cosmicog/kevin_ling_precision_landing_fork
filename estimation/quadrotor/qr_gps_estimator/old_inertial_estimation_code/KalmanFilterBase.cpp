/**
 * Implementation of the KalmanFilterBase class.
 *
 * Author: K.Ling
 * Dec 2013
 *
 * WAVELab, University of Waterloo
 */

#include "KalmanFilterBase.hpp"
#include <iostream>

using namespace Eigen;

KalmanFilterBase::KalmanFilterBase(int num_states)
{
  _mean = VectorXd::Zero(num_states);
  _sigma = MatrixXd::Zero(num_states, num_states);
}


VectorXd KalmanFilterBase::getMeanStates()
{
  return _mean;
}


MatrixXd KalmanFilterBase::getStateCovariances()
{
  return _sigma;
}


void KalmanFilterBase::performPredictionUpdate(MatrixXd& matr_A,
    VectorXd& predicted_state, MatrixXd& process_noise)
{
  if (_mean.rows() != predicted_state.rows())
  {
    std::cerr << "KalmanFilterBase Warning: Changing size of state vector " <<
        "in a prediction update!" << std::endl;
  }

  _mean = predicted_state;
  _sigma = matr_A*_sigma*matr_A.transpose() + process_noise;
}


void KalmanFilterBase::performMeasurementUpdate(MatrixXd& matr_C, 
    VectorXd& measurements, VectorXd& pred_measurements,
    MatrixXd& measurement_noise)
{
  MatrixXd temp(matr_C.rows(), matr_C.rows());
  temp = matr_C * _sigma * matr_C.transpose() + measurement_noise;

  MatrixXd matr_K(matr_C.cols(), matr_C.rows());
  matr_K = _sigma * matr_C.transpose() * temp.inverse();

  _mean = _mean + matr_K*(measurements - pred_measurements);
  _sigma = ( MatrixXd::Identity(_sigma.rows(), _sigma.rows())
      - matr_K*matr_C ) * _sigma;
}


void KalmanFilterBase::setStateEstimates(VectorXd& new_state)
{
  if (_mean.rows() != new_state.rows())
  {
    std::cerr << "KalmanFilterBase Warning: Changing size of state vector " <<
        "when reinitializing!" << std::endl;
  }

  _mean = new_state;
}


