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
 * File: pelican_utils.cc
 * Desc: Source definitions for a collection of utility functions for working
 *       with the Asctec Pelican platform.
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

#include <wave_pelican/pelican_utils.h>

#define _USE_MATH_DEFINES
#include <math.h>

#include <ros/ros.h>
#include <asctec_hl_comm/mav_status.h>

const double PelicanUtils::MASS_KG_DEFAULT = 1.640;

const double PelicanUtils::THRUST_CURVE_A_DEFAULT = 2.826261e-6;
const double PelicanUtils::THRUST_CURVE_B_DEFAULT = 1.261904445e-3;
const double PelicanUtils::THRUST_CURVE_C_DEFAULT = 1.25717613;

const double PelicanUtils::ANGVEL_TO_RAD_PER_SECOND_ = 0.0154;
const double PelicanUtils::ANGLE_TO_RAD_ = 1.745329251994330e-05;
const double PelicanUtils::YAW_RATE_TO_PELICAN_COMMAND = -1.0;
const double PelicanUtils::MAX_THRUST_COMMAND = 4000.0;
const double PelicanUtils::THRUST_NORMALIZER = 4095.0;
const double PelicanUtils::MIN_THRUST_COMMAND = 200.0;

const std::string PelicanUtils::MOTOR_STATUS_RUNNING = "running";

PelicanUtils& PelicanUtils::getInstance() {
  static PelicanUtils instance;
  return instance;
}

PelicanUtils::PelicanUtils() {
  ros::NodeHandle nh;
  bool using_atleast_one_default_thrust_curve_param = false;
  if (!nh.getParam("wave/thrust_curve_a", thrust_curve_a_)) {
    using_atleast_one_default_thrust_curve_param = true;
  }
  if (!nh.getParam("wave/thrust_curve_b", thrust_curve_b_)) {
    using_atleast_one_default_thrust_curve_param = true;
  }
  if (!nh.getParam("wave/thrust_curve_c", thrust_curve_c_)) {
    using_atleast_one_default_thrust_curve_param = true;
  }
  if (using_atleast_one_default_thrust_curve_param) {
    ROS_WARN("PelicanUtils: Atleast one thrust curve parameter is not set, "
             "using all default thrust curve values.");

    thrust_curve_a_ = THRUST_CURVE_A_DEFAULT;
    thrust_curve_b_ = THRUST_CURVE_B_DEFAULT;
    thrust_curve_c_ = THRUST_CURVE_C_DEFAULT;
  }

  if (!nh.getParam("wave/qr_mass", mass_kg)) {
    mass_kg = MASS_KG_DEFAULT;
    ROS_WARN("PelicanUtils: wave/qr_mass parameter not set. Using default.");
  }
}


double PelicanUtils::thrustCommandToCollectiveForce(double thrust_command) {

  if (0 >= thrust_command || thrust_command > 1) {
    return 0.0;
  }
  thrust_command *= THRUST_NORMALIZER;

  // Quadratic formula to map thrust from 0..4095 to Newtons.
  return (thrust_curve_a_*pow(thrust_command, 2) +
      thrust_curve_b_*thrust_command +
      thrust_curve_c_);
}


float PelicanUtils::collectiveForceToThrustCommand(double thrust_newtons) {
  thrust_newtons = fmax(0.0, thrust_newtons);

  // Quadratic formula.
  double output = -1*thrust_curve_b_ +
      sqrt(pow(thrust_curve_b_, 2) -
      (4*thrust_curve_a_*(thrust_curve_c_ - thrust_newtons)));
  output = output / (2 * thrust_curve_a_);

  output = fmax(MIN_THRUST_COMMAND, output);
  output = fmin(MAX_THRUST_COMMAND, output);
  output = output / THRUST_NORMALIZER;
  return static_cast<float>(output);
}

bool PelicanUtils::checkAutonomousMode(const asctec_hl_comm::mav_status& msg) {
  bool motors_running = PelicanUtils::MOTOR_STATUS_RUNNING.compare(
                            msg.motor_status) == 0;
  return motors_running && msg.serial_interface_enabled;
}

