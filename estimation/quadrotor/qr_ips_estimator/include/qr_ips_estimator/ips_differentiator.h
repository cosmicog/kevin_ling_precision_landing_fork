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
 * File: ips_differentiator.h
 * Desc: Prototypes for a lass that differentiates indoor position measurements
 *  and low pass filters them for a less noisy velocity.
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


#ifndef QR_IPS_ESTIMATOR_INCLUDE_QR_IPS_ESTIMATOR_IPS_DIFFERENTIATOR_H_
#define QR_IPS_ESTIMATOR_INCLUDE_QR_IPS_ESTIMATOR_IPS_DIFFERENTIATOR_H_

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>

#include <wave_utils/wave_dsp.h>

/**
 * Subscribes to measurements from the indoor positioning system (IPS) and 
 * performs differentation and low-pass filtering to obtain linear velocity
 * states.
 */
class IPSDifferentiator {
  public:
    /**
     * Constructor.
     *
     * @param fcut Cutoff frequency for low-pass filters on derived velocities.
     */
    IPSDifferentiator(double fcut);

    ~IPSDifferentiator() {/*Empty Destructor*/}

    /**
     * Process the IPS measurements. Performs a single difference with a
     * low-pass to obtain linear velocities.
     *
     * Internal variables keep track of time between calls to this function
     * for differentiation.
     *
     * @param[in] measurement The latest IPS measurement.
     */
    void processIPSMeasurements(const geometry_msgs::Pose& measurement);

    double get_x_velocity() {
      return x_velocity_;
    }

    double get_y_velocity() {
      return y_velocity_;
    }
    
    double get_z_velocity() {
      return z_velocity_;
    }
    
  private:
    // Linear velocities from differentiation with LPF, m/s
    double x_velocity_;
    double y_velocity_;
    double z_velocity_;

    // Cutoff frequency for low-pass filtering.
    double cutoff_frequency_;

    // Variables to keep track of previous data for differentiation.
    ros::Time last_ips_measurement_time_;
    geometry_msgs::Pose last_ips_measurement_;
};

#endif // QR_IPS_ESTIMATOR_INCLUDE_QR_IPS_ESTIMATOR_IPS_DIFFERENTIATOR_H_


