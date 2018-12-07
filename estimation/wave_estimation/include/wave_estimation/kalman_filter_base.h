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
 * File: kalman_filter_base.h
 * Desc: Header file for the KalmanFilterBase class, an implementation of the
 *    Kalman filter equations using the Eigen3 library.*
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

#ifndef WAVE_ESTIMATION_INCLUDE_WAVE_ESTIMATION_KALMAN_FILTER_BASE_H_
#define WAVE_ESTIMATION_INCLUDE_WAVE_ESTIMATION_KALMAN_FILTER_BASE_H_

#include <Eigen/Core>
#include <Eigen/LU>


/**
 * Implementation of Kalman Filter equations and encapsulation of state
 * estimates and covariances.
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
    Eigen::VectorXd mean_;

    /**
     * Covariances for the current mean estimates.
     */
    Eigen::MatrixXd sigma_;
};

#endif // WAVE_ESTIMATION_INCLUDE_WAVE_ESTIMATION_KALMAN_FILTER_BASE_H_
