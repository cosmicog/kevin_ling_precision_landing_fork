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
 * File: height_controller.h
 * Desc: Prototypes for the HeightController class.
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


#ifndef QR_OUTERLOOP_PID_INCLUDE_QR_OUTERLOOP_PID_HEIGHT_CONTROLLER_H_
#define QR_OUTERLOOP_PID_INCLUDE_QR_OUTERLOOP_PID_HEIGHT_CONTROLLER_H_

#include <string>

#include <ros/ros.h>

#include <wave_msgs/QRStateEstimate.h>
#include <wave_msgs/QRControl.h>

#include <wave_control/pid_controller_base.h>

/**
 * Altitude controller for a quadrotor.
 */
class HeightController
{
  public:
    /**
     * Constructor. Initializes variables from ROS parameters.
     *
     * @param[in] nh Public NodeHandle of current node.
     * @param[in] mass Mass of the quadrotor system, in kg.
     */
    HeightController(ros::NodeHandle nh, double mass);

    /**
     * Runs the controller and calculates the desired thrust in Newtons.
     *
     * @param[in] inertial_state Current quadrotor state estimates.
     * @param[in] ref Commands for the quadrotor.
     * @return Desired thrust in Newtons.
     */
    double run(const wave_msgs::QRStateEstimate& inertial_state,
               const wave_msgs::QRControl& ref);

    /**
     * Resets the altitude error integrator. 
     */
    void resetIntegrator();


  private:
    PIDControllerBase controller_base_;

    // Mass of the quadrotor system, in kg.
    double mass_;

    // Gain is in: m/s/s / m
    static const double DEFAULT_HEIGHT_KP;
    static const double DEFAULT_HEIGHT_KI;
    static const double DEFAULT_HEIGHT_KD;

    static const double DEFAULT_HEIGHT_ERROR_LIMIT_POSITIVE;
    double height_error_limit_positive_;
    static const double DEFAULT_HEIGHT_ERROR_LIMIT_NEGATIVE;
    double height_error_limit_negative_;

    // Thrust trim, to correct for any errors in the nominal hover thrust.
    double trim_;
    
    static const std::string HEIGHT_CONTROLLER_NAME;
};

#endif // QR_OUTERLOOP_PID_INCLUDE_QR_OUTERLOOP_PID_HEIGHT_CONTROLLER_H_

