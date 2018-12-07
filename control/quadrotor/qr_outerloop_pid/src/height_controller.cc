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
 * File: height_controller.cc
 * Desc: Source definitions for the HeightController class.
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

#include <qr_outerloop_pid/height_controller.h>

#include <wave_utils/wave_math.h>

// Gain is in: m/s/s / m
const double HeightController::DEFAULT_HEIGHT_KP = 2.3927;
const double HeightController::DEFAULT_HEIGHT_KI = 0.1196;
const double HeightController::DEFAULT_HEIGHT_KD = 1.9141;

const double HeightController::DEFAULT_HEIGHT_ERROR_LIMIT_POSITIVE = 0.80;
const double HeightController::DEFAULT_HEIGHT_ERROR_LIMIT_NEGATIVE = -0.70;

const std::string HeightController::HEIGHT_CONTROLLER_NAME =
    "height";

HeightController::HeightController(ros::NodeHandle nh,
                                   double mass) :
                                   mass_(mass),
                                   controller_base_(nh,
                                                    DEFAULT_HEIGHT_KP,
                                                    DEFAULT_HEIGHT_KI,
                                                    DEFAULT_HEIGHT_KD,
                                                    HEIGHT_CONTROLLER_NAME) {

  nh.param<double>(HEIGHT_CONTROLLER_NAME + "/error_limit_positive",
                   height_error_limit_positive_,
                   DEFAULT_HEIGHT_ERROR_LIMIT_POSITIVE);
  nh.param<double>(HEIGHT_CONTROLLER_NAME + "/error_limit_negative",
                   height_error_limit_negative_,
                   DEFAULT_HEIGHT_ERROR_LIMIT_NEGATIVE);
  nh.param<double>(HEIGHT_CONTROLLER_NAME + "/trim", trim_, 0);
}


void HeightController::resetIntegrator() {
  controller_base_.resetIntegrator();
}


double HeightController::run(const wave_msgs::QRStateEstimate& inertial_state,
                             const wave_msgs::QRControl& ref) {

  double position_error, velocity_error;
  switch (ref.type) {
    default:
      ROS_INFO("%s: Unsupported QRControl type. Treating command as "
               "inertial command.", HEIGHT_CONTROLLER_NAME.c_str());
      /* fall through */
    case wave_msgs::QRControl::CMD_INERTIAL_POSITION:
      /* fall through */
    case wave_msgs::QRControl::CMD_RELATIVE_POSITION:
      position_error = ref.z - inertial_state.z;
      velocity_error = 0.0 - inertial_state.dz;
      break;

    case wave_msgs::QRControl::CMD_INERTIAL_TRAJECTORY:
      position_error = ref.z - inertial_state.z;
      velocity_error = ref.dz - inertial_state.dz;
      break;
  }
  position_error = wave_utils::saturate(position_error,
                                        height_error_limit_positive_,
                                        height_error_limit_negative_);

  // Output is a desired acceleration
  double output = controller_base_.run(position_error, velocity_error);

  // Add feedforward for gravity and the trim
  output += wave_utils::GRAVITY_MSS + trim_;

  // Convert to a force
  output *= mass_;

  // Compensate for tilt.
  double roll_factor = cos(inertial_state.roll);
  double pitch_factor = cos(inertial_state.pitch);
  output = output / (roll_factor * pitch_factor);

  return output;
}


