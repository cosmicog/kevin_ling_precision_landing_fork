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
 * File: outerloop_pid_controller.cc
 * Desc: Source definitions for OuterloopPIDcontroller class.
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

#include <qr_outerloop_pid/outerloop_pid_controller.h>

#include <qr_outerloop_pid/SetTranslationControllerStatus.h>

#include <qr_outerloop_pid/height_controller.h>
#include <qr_outerloop_pid/translation_controller.h>
#include <qr_outerloop_pid/yaw_controller.h>

// TODO: Add watchdog to verify messages are not arriving too fast or too slow.

const double OuterloopPIDController::DEFAULT_MAX_CONTROL_RATE = 100; // Hz

OuterloopPIDController::OuterloopPIDController(
    ros::NodeHandle nh,
    double mass,
    ControlMessagePublisher& message_publisher) :
    message_publisher_(message_publisher),
    inertial_estimate_rx_count_(0),
    command_rx_count_(0),
    height_controller_(nh, mass),
    yaw_controller_(nh),
    translation_controller_(nh, mass) {

  relative_estimate_subscriber_ = nh.subscribe("wave/qr_relative_estimate", 1,
      &OuterloopPIDController::relativeEstimateSubscriberCallback, this);

  inertial_estimate_subscriber_ = nh.subscribe("wave/qr_inertial_estimate", 1,
      &OuterloopPIDController::inertialEstimateSubscriberCallback, this);

  command_subscriber_ = nh.subscribe("wave/qr_command", 1,
      &OuterloopPIDController::commandSubscriberCallback, this);

  translation_controller_service_ = nh.advertiseService(
      "wave/translation_controller_status",
      &OuterloopPIDController::translationControllerStatusServiceCallback,
      this);

  double max_control_rate;
  nh.param<double>("wave/max_control_rate", max_control_rate,
                      DEFAULT_MAX_CONTROL_RATE);
  min_control_period_ = 1.0 / max_control_rate;

  controller_enabled_ = false;
  translation_control_enabled_ = true;
}

void OuterloopPIDController::stopController() {
  controller_enabled_ = false;
}

void OuterloopPIDController::startController() {
  if (!controller_enabled_) {
    height_controller_.resetIntegrator();
    yaw_controller_.resetIntegrator();
    translation_controller_.resetIntegrator();
    controller_enabled_ = true;
  }
}


void OuterloopPIDController::run() {
  if (!controller_enabled_ ||
      !(inertial_estimate_rx_count_ & relative_estimate_rx_count_) || 
      !command_rx_count_) {
    return;
  }

  // Throttle control rate.
  static ros::Time last_run_time(0);
  ros::Time curr_time = ros::Time::now();
  if ((curr_time - last_run_time).toSec() < min_control_period_) {
    return;
  }
  last_run_time = curr_time;
  
  double thrust = height_controller_.run(latest_inertial_estimate_,
                                         latest_command_);
  double yaw_rate = yaw_controller_.run(latest_inertial_estimate_,
                                        latest_command_);

  RollPitchCommand attitude;
  if (!translation_control_enabled_) {
    attitude.roll = 0;
    attitude.pitch = 0;
  } else if (wave_msgs::QRControl::CMD_RELATIVE_POSITION ==
             latest_command_.type) {
    if (!relative_estimate_rx_count_) {
      ROS_ERROR_THROTTLE(5, "OuterloopPIDController: No relative position "
                         "estimates for relative command!");
      attitude.roll = 0;
      attitude.pitch = 0;
    } else {
      attitude = translation_controller_.run(latest_relative_estimate_,
                                             latest_command_,
                                             thrust);
    }
  } else {
    if (!inertial_estimate_rx_count_) {
      ROS_ERROR_THROTTLE(5, "OuterloopPIDController: No inertial position "
                         "estimates for inertial command!");
      attitude.roll = 0;
      attitude.pitch = 0;
    } else {
      attitude = translation_controller_.run(latest_inertial_estimate_,
                                             latest_command_,
                                             thrust);
    }
  }
  message_publisher_.publishControlMessageCallback(thrust,
                                                   attitude.roll,
                                                   attitude.pitch,
                                                   yaw_rate);
}


void OuterloopPIDController::inertialEstimateSubscriberCallback(
    const wave_msgs::QRStateEstimate& msg) {
  inertial_estimate_rx_count_++;
  latest_inertial_estimate_ = msg;
  run();
}


void OuterloopPIDController::relativeEstimateSubscriberCallback(
    const wave_msgs::QRStateEstimate& msg) {
  relative_estimate_rx_count_++;
  latest_relative_estimate_ = msg;
  run();
}


void OuterloopPIDController::commandSubscriberCallback(
    const wave_msgs::QRControl& msg) {
  command_rx_count_++;
  latest_command_ = msg;
}

bool OuterloopPIDController::translationControllerStatusServiceCallback(
    qr_outerloop_pid::SetTranslationControllerStatus::Request& request,
    qr_outerloop_pid::SetTranslationControllerStatus::Response& response) {
  translation_control_enabled_ = request.enable;
  response.running = translation_control_enabled_;
  return true;
}

