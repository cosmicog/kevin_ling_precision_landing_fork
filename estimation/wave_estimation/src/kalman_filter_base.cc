/* 
 * ########################################################################    
 *                   ____
 *                  /    \
 *     ____         \____/
 *    /    \________//      ______   
 *    \____/--/      \_____/     \\
 *            \______/----/      // __        __  __  __      __  ______ 
 *            //          \_____//  \ \  /\  / / /  \ \ \    / / / ____/  
 *       ____//                      \ \/  \/ / / /\ \ \ \  / / / /__ 
 *      /      \\                     \  /\  / / /  \ \ \ \/ / / /____   
 *     /       //                      \/  \/ /_/    \_\ \__/ /______/ 
 *     \______//                     LABORATORY
 *
 * ########################################################################
 *
 * File: kalman_filter_base.cc
 * Desc: Source file for the KalmanFilterBase class, an implementation of the
 *    Kalman filter equations using the Eigen3 library.
 * Auth: Kevin Ling
 *
 * You can contact the author at <kling at uwaterloo dot ca>.
 *
 * Copyright (c) 2014, Waterloo Autonomous Vehicles Laboratory (WAVELab),
 * University of Waterloo.
 *
 * All Rights Reserved
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *   * Neither the name of the Waterloo Autonomous Vehicles Laboratory nor the
 *     names of its contributors may be used to endorse or promote products
 *     derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE WATERLOO AUTONOMOUS VEHICLES LABORATORY
 * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR 
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF 
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <wave_estimation/kalman_filter_base.h>

#include <ros/ros.h>

using namespace Eigen;

KalmanFilterBase::KalmanFilterBase(int num_states) {
  mean_ = VectorXd::Zero(num_states);
  sigma_ = MatrixXd::Zero(num_states, num_states);
}


VectorXd KalmanFilterBase::getMeanStates() {
  return mean_;
}


MatrixXd KalmanFilterBase::getStateCovariances() {
  return sigma_;
}


void KalmanFilterBase::performPredictionUpdate(MatrixXd& matr_A,
    VectorXd& predicted_state, MatrixXd& process_noise) {
  if (mean_.rows() != predicted_state.rows()) {
    ROS_WARN_THROTTLE(1, "KalmanFilterBase Warning: Changing size of state "
        "vector in a prediction update!");
  }
  mean_ = predicted_state;
  sigma_ = matr_A*sigma_*matr_A.transpose() + process_noise;
}


void KalmanFilterBase::performMeasurementUpdate(MatrixXd& matr_C, 
    VectorXd& measurements, VectorXd& pred_measurements,
    MatrixXd& measurement_noise) {
  MatrixXd temp(matr_C.rows(), matr_C.rows());
  temp = matr_C * sigma_ * matr_C.transpose() + measurement_noise;

  MatrixXd matr_K(matr_C.cols(), matr_C.rows());
  matr_K = sigma_ * matr_C.transpose() * temp.inverse();

  mean_ = mean_ + matr_K*(measurements - pred_measurements);
  sigma_ = ( MatrixXd::Identity(sigma_.rows(), sigma_.rows())
      - matr_K*matr_C ) * sigma_;
}


void KalmanFilterBase::setStateEstimates(VectorXd& new_state)
{
  if (mean_.rows() != new_state.rows()) {
    ROS_WARN_THROTTLE(1, "KalmanFilterBase Warning: Changing size of state "
        "vector when reinitializing!");
  }
  mean_ = new_state;
}


