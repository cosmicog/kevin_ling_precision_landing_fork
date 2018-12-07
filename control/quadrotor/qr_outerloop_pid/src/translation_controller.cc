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
 * File: translation_controller.cc
 * Desc: Source definitions for the TranslationController class.
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

#include <qr_outerloop_pid/translation_controller.h>

#include <ros/ros.h>

#include <wave_utils/wave_math.h>
#include <math.h>
#include <wave_msgs/QRPositionPid.h>

#include <wave_pelican/pelican_utils.h>

const double TranslationController::DEFAULT_TRANSLATION_KP = 3.3;
const double TranslationController::DEFAULT_TRANSLATION_KI = 0.3;
const double TranslationController::DEFAULT_TRANSLATION_KD = 1.9;

// Max roll or pitch angle, in radians
const double TranslationController::DEFAULT_MAX_ROLL_PITCH_ANGLE =
    0.523598776; // 30 deg

const double TranslationController::DEFAULT_INTEGRATOR_LIMIT = 2.00;

const double TranslationController::DEFAULT_MAX_TRANSLATION_ERROR = 0.75;
const double TranslationController::DEFAULT_MAX_TRANSLATION_RATE_ERROR = 4.00;

const std::string TranslationController::TRANSLATION_CONTROLLER_NAME =
    "translation";

TranslationController::TranslationController(ros::NodeHandle nh,
                                             double mass) :
                                             mass_(mass),
                                             x_controller_base_(nh,
                                                DEFAULT_TRANSLATION_KP,
                                                DEFAULT_TRANSLATION_KI,
                                                DEFAULT_TRANSLATION_KD,
                                                TRANSLATION_CONTROLLER_NAME +
                                                  "_x"),
                                             y_controller_base_(nh,
                                                DEFAULT_TRANSLATION_KP,
                                                DEFAULT_TRANSLATION_KI,
                                                DEFAULT_TRANSLATION_KD,
                                                TRANSLATION_CONTROLLER_NAME +
                                                  "_y") {

  nh.param<double>(TRANSLATION_CONTROLLER_NAME + "/integrator_limit",
                 integrator_limit_,
                 DEFAULT_INTEGRATOR_LIMIT);
  x_controller_base_.setIntegratorLimits(integrator_limit_,
                                         -integrator_limit_);
  y_controller_base_.setIntegratorLimits(integrator_limit_,
                                         -integrator_limit_);

  nh.param<double>(TRANSLATION_CONTROLLER_NAME + "/max_angle",
                   rp_max_angle_deg_,
                   DEFAULT_MAX_ROLL_PITCH_ANGLE);

  nh.param<double>(TRANSLATION_CONTROLLER_NAME + "/max_translation_error",
                   max_translation_error_,
                   DEFAULT_MAX_TRANSLATION_ERROR);

  nh.param<double>(TRANSLATION_CONTROLLER_NAME + "/max_translation_rate_error",
                   max_translation_rate_error_,
                   DEFAULT_MAX_TRANSLATION_RATE_ERROR);
}


RollPitchCommand TranslationController::run(
    const wave_msgs::QRStateEstimate& state,
    const wave_msgs::QRControl& ref,
    double thrust) {

  double x_error, y_error, x_rate_error, y_rate_error;
  bool relative_command = false;

  switch (ref.type) {
    case wave_msgs::QRControl::CMD_INERTIAL_POSITION:
      x_error = ref.x - state.x;
      x_rate_error = 0 - state.dx;
      y_error = ref.y - state.y;
      y_rate_error = 0 - state.dy;
      break;

    case wave_msgs::QRControl::CMD_INERTIAL_TRAJECTORY:
      /* fall through */
    case wave_msgs::QRControl::CMD_RELATIVE_POSITION:
      x_error = ref.x - state.x;
      x_rate_error = ref.dx - state.dx;
      y_error = ref.y - state.y;
      y_rate_error = ref.dy - state.dy;
      break;

    default:
      ROS_ERROR("%s: Unsupported QRControl type. Commanding neutral attitudes.",
               TRANSLATION_CONTROLLER_NAME.c_str());
      x_error = 0;
      y_error = 0;
      resetIntegrator();
      break;
  }

  x_error = wave_utils::saturate(x_error, max_translation_error_,
                                -max_translation_error_);
  y_error = wave_utils::saturate(y_error, max_translation_error_,
                                -max_translation_error_);
  x_rate_error = wave_utils::saturate(x_rate_error, max_translation_rate_error_,
                                -max_translation_rate_error_);
  y_rate_error = wave_utils::saturate(y_rate_error, max_translation_rate_error_,
                                -max_translation_rate_error_);

  double x_inertial_accel = x_controller_base_.run(x_error, x_rate_error);
  double y_inertial_accel = y_controller_base_.run(y_error, y_rate_error);

  double x_body_accel, y_body_accel;

  if (relative_command) {
    x_body_accel = x_inertial_accel;
    y_body_accel = y_inertial_accel;
  } else {
    // TODO: Refactor this to use a proper rotation function
    Eigen::Vector2d body_accel_vect(x_body_accel, y_body_accel);
    x_body_accel = cos(state.yaw)*x_inertial_accel +
          sin(state.yaw)*y_inertial_accel;
    y_body_accel = cos(state.yaw)*y_inertial_accel -
          sin(state.yaw)*x_inertial_accel;

    /*
    Eigen::Vector2d body_accel_vect(x_body_accel, y_body_accel);
    Eigen::Vector2d rotated_vector = wave_utils::rotateByYawInR2(body_accel_vect, state.yaw);
    x_body_accel = rotated_vector(0);
    y_body_accel = rotated_vector(1);
    */

  }

  RollPitchCommand output_angles = mapAccelToAngle(x_body_accel,
                                                   y_body_accel,
                                                   thrust);

  output_angles.roll = wave_utils::saturate(output_angles.roll,
      rp_max_angle_deg_, -rp_max_angle_deg_);
  output_angles.pitch = wave_utils::saturate(output_angles.pitch,
      rp_max_angle_deg_, -rp_max_angle_deg_);

  return output_angles;
}


void TranslationController::resetIntegrator() {
  x_controller_base_.resetIntegrator();
  y_controller_base_.resetIntegrator();
}


RollPitchCommand TranslationController::mapAccelToAngle(double x_body_acc,
                                                        double y_body_acc,
                                                        double thrust) {
  double thrust_acc = thrust / mass_;

  // Fully right handed coordinate system with Z pointing up and X forward.
  // +pitch -> qr accelerates back
  // +roll -> qr accelerates to the right
  double x,y;
  if (0 == thrust_acc) {
    // Don't divide by zero, just do nothing.
    x = 0;
    y = 0;
  } else {
    x = -x_body_acc/thrust_acc;
    y = y_body_acc/thrust_acc;
  }

  RollPitchCommand command;
  command.roll = asin(y);
  command.pitch = asin(x / cos(command.roll));

  return command;
}

