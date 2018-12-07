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
 * File: yaw_controller.cc
 * Desc: Source definitions for the YawController class.
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

#include <qr_outerloop_pid/yaw_controller.h>

#include <wave_utils/wave_math.h>

// Gain is in: rad/s / rad
const double YawController::DEFAULT_YAW_KP = 0.8;
const double YawController::DEFAULT_YAW_KI = 0.01;
const double YawController::DEFAULT_YAW_KD = 0.3; 

const double YawController::INTEGRATOR_LIMIT = 2.0; // rad/s
const double YawController::MAX_COMMAND = 4.0; // rad/s

const std::string YawController::YAW_CONTROLLER_NAME = "yaw";

YawController::YawController(ros::NodeHandle nh) : controller_base_(nh,
                                                      DEFAULT_YAW_KP,
                                                      DEFAULT_YAW_KI,
                                                      DEFAULT_YAW_KD,
                                                      YAW_CONTROLLER_NAME) {
  controller_base_.setIntegratorLimits(INTEGRATOR_LIMIT,
                                       -INTEGRATOR_LIMIT);

  controller_base_.setOutputSaturationLimits(MAX_COMMAND, -MAX_COMMAND);
}


double YawController::run(const wave_msgs::QRStateEstimate& state,
                          const wave_msgs::QRControl& ref) {
  double position_error, velocity_error;

  switch (ref.type) {
    default:
      ROS_INFO("%s: Unsupported QRControl type. Treating command as "
               "inertial command.", YAW_CONTROLLER_NAME.c_str());
      /* fall through */
    case wave_msgs::QRControl::CMD_INERTIAL_POSITION:
      /* fall through */
    case wave_msgs::QRControl::CMD_RELATIVE_POSITION:
      position_error = wave_utils::unwrapAngle(ref.yaw - state.yaw);
      velocity_error = 0.0 - state.dyaw;
      break;

    case wave_msgs::QRControl::CMD_INERTIAL_TRAJECTORY:
      position_error = wave_utils::unwrapAngle(ref.yaw - state.yaw);
      velocity_error = ref.dyaw - state.dyaw;
      break;
  }

  return controller_base_.run(position_error, velocity_error);
}

void YawController::resetIntegrator() {
  controller_base_.resetIntegrator();
}


