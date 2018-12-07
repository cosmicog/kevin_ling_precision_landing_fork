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
 * File: pid_controller_base.cc
 * Desc: Source file for the PIDControllerBase class.
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

#include <wave_control/pid_controller_base.h>

#include <math.h>
#include <cmath>

#include <ros/ros.h>

#include <wave_msgs/PidLog.h>
#include <wave_utils/wave_math.h>



PIDControllerBase::PIDControllerBase(ros::NodeHandle nh, double default_kp,
                                     double default_ki, double default_kd,
                                     std::string controller_name) :
                                     controller_name_(controller_name),
                                     integrator_(0),
                                     timeout_duration_(1.0),
                                     output_lim1_(0),
                                     output_lim2_(0),
                                     output_saturator_enabled_(false),
                                     integrator_lim1_(0),
                                     integrator_lim2_(0),
                                     integrator_saturator_enabled_(false),
                                     integrator_freshly_set_(false),
                                     nh_(nh) {

  if (!nh_.getParamCached(controller_name_ + "/Kp", kp_)) {
    nh_.setParam(controller_name_ + "/Kp", default_kp);
    kp_ = default_kp;
  }
  if (!nh_.getParamCached(controller_name_ + "/Ki", ki_)) {
    nh_.setParam(controller_name_ + "/Ki", default_ki);
    ki_ = default_ki;
  }
  if (!nh_.getParamCached(controller_name_ + "/Kd", kd_)) {
    nh_.setParam(controller_name_ + "/Kd", default_kd);
    kd_ = default_kd;
  }

  ROS_INFO("Controller %s intialized with gains: P-%f, I-%f, D-%f",
      controller_name_.c_str(), kp_, ki_, kd_);

  nh_.param<bool>("enable_pid_logging", logging_enabled_, false);
  if (logging_enabled_) {
    controller_log_pub_ = nh_.advertise<wave_msgs::PidLog>(
        controller_name_ + "/pid_log", 1);
  }

  last_run_time_ = ros::Time(0);
}


void PIDControllerBase::setOutputSaturationLimits(double lim1, double lim2) {
  output_lim1_ = lim1;
  output_lim2_ = lim2;
  output_saturator_enabled_ = true;
}


void PIDControllerBase::setIntegratorLimits(double lim1, double lim2) {
  integrator_lim1_ = lim1;
  integrator_lim2_ = lim2;
  integrator_saturator_enabled_ = true;
}


void PIDControllerBase::setTimeoutDuration(double duration) {
  if (duration < 0.0) {
    ROS_WARN("Controller %s: Timeout duration must be positive. Timeout "
        "unchanged.", controller_name_.c_str());
  } else {
    timeout_duration_ = duration;
  }
}


void PIDControllerBase::resetIntegrator() {
  integrator_ = 0;
}


void PIDControllerBase::setIntegrator(double integrator) {
  integrator_freshly_set_ = true;
  integrator_ = integrator;
}


double PIDControllerBase::run(double proportional_error,
                              double derivative_error) {
  // Check for gain changes.
  nh_.getParamCached(controller_name_ + "/Kp", kp_);
  nh_.getParamCached(controller_name_ + "/Ki", ki_);
  nh_.getParamCached(controller_name_ + "/Kd", kd_);

  bool proportional_error_used = (kp_ != 0) || (ki_ != 0);
  if ((!std::isfinite(proportional_error) && proportional_error_used) || 
      (!std::isfinite(derivative_error) && (kd_ != 0))) {
    ROS_WARN_THROTTLE(1, "Controller %s: Control error of NaN received, "
                      "outputting 0.", controller_name_.c_str());
    return 0;
  }

  ros::Time current_time = ros::Time::now();
  double dt = (current_time - last_run_time_).toSec();
  if (dt > timeout_duration_) {
    if (!integrator_freshly_set_) {
      // Timeout tripped, which means the controller has just started from being
      // off, or estimates/commands are coming in too slowly. Either way, the
      // error integral is stale and doesn't apply anymore.
      resetIntegrator();
    }
    // If integrator was freshly set, do nothing. Do not integrate here!
  } else {
    integrator_ += proportional_error * ki_ * dt;

    if (integrator_saturator_enabled_) {
      integrator_ = wave_utils::saturate(integrator_, integrator_lim1_,
                                   integrator_lim2_);
    }
    integrator_freshly_set_ = false;
  }
  last_run_time_ = current_time;

  double p = proportional_error*kp_;
  double d = derivative_error*kd_;

  double output = p + integrator_ + d;

  if (output_saturator_enabled_) {
    output = wave_utils::saturate(output, output_lim1_, output_lim2_);
  }

  if (logging_enabled_) {
    wave_msgs::PidLog msg;
    msg.header.stamp = ros::Time::now();
    msg.p = p;
    msg.i = integrator_;
    msg.d = d;
    msg.output = output;
    controller_log_pub_.publish(msg);
  }

  return output;
}


