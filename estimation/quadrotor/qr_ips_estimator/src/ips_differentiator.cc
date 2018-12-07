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
 * File: ips_differentiator.cc
 * Desc: Class that differentiates indoor position measurements and low pass
 *  filters them for a less noisy velocity.
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

#include <qr_ips_estimator/ips_differentiator.h>

IPSDifferentiator::IPSDifferentiator(double fcut) :
                                     x_velocity_(0),
                                     y_velocity_(0),
                                     z_velocity_(0),
                                     last_ips_measurement_time_(0) {
  cutoff_frequency_ = fcut;
}


void IPSDifferentiator::processIPSMeasurements(
    const geometry_msgs::Pose& measurement) {

  ros::Time current_time = ros::Time::now();
  double dt = (current_time - last_ips_measurement_time_).toSec();
  if (dt > 0.5) {
    // Timeout and reset if it takes too long between measurements.
    x_velocity_ = 0;
    y_velocity_ = 0;
    z_velocity_ = 0;
  } else {
    double new_differentiated_x = (measurement.position.x -
                                   last_ips_measurement_.position.x) / dt;
    double new_differentiated_y = (measurement.position.y -
                                   last_ips_measurement_.position.y) / dt;
    double new_differentiated_z = (measurement.position.z -
                                   last_ips_measurement_.position.z) / dt;

    wave_utils::updateLowPassFilter(x_velocity_, new_differentiated_x,
                                  cutoff_frequency_, dt);
    wave_utils::updateLowPassFilter(y_velocity_, new_differentiated_y,
                                  cutoff_frequency_, dt);
    wave_utils::updateLowPassFilter(z_velocity_, new_differentiated_z,
                                  cutoff_frequency_, dt);
  }
  last_ips_measurement_time_ = current_time;
  last_ips_measurement_ = measurement;
}

