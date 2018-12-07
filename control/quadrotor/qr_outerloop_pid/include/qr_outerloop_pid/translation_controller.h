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
 * File: translation_controller.h
 * Desc: Prototypes for the TranslationController class.
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

#ifndef QR_OUTERLOOP_PID_INCLUDE_QR_OUTERLOOP_PID_TRANSLATION_CONTROLLER_H_ 
#define QR_OUTERLOOP_PID_INCLUDE_QR_OUTERLOOP_PID_TRANSLATION_CONTROLLER_H_

#include <ros/ros.h>

#include <wave_msgs/QRStateEstimate.h>
#include <wave_msgs/QRControl.h>

#include <wave_control/pid_controller_base.h>

/**
 * Struct to return the commanded roll and pitch.
 */
typedef struct RP_COMMAND_ {
  double roll;
  double pitch;
} RollPitchCommand;


/**
 * Translational (x-y plane) position controller for the Asctec Pelican.
 */
class TranslationController
{
  public:
    /**
     * Constructor. Initializes variables from ROS parameters.
     *
     * @param[in] nh NodeHandle to check for ROS parameters.
     * @param[in] mass The mass of the quadrotor system, in kg.
     */
    TranslationController(ros::NodeHandle nh, double mass);

    /**
     * Runs the controller and calculates the commanded angles.
     *
     * @param[in] state Current quadrotor states in the the same frame as the
     *    ref commands.
     * @param[in] ref Commands to track.
     * @param[in] thrust Current quadrotor thrust command, in Newtons.
     * @return Struct with the commanded roll and pitch commands, in radians.
     */
    RollPitchCommand run(const wave_msgs::QRStateEstimate& state,
                         const wave_msgs::QRControl& ref,
                         double thrust);

    /**
     * Resets the x and y position error integrators.
     */
    void resetIntegrator();


    //TODO Add integrator rotations

  private:
    /**
     * Helper function to map the desired body frame accelerations to angles.
     *
     * @param[in] x_body_acc Desired x-acceleration.
     * @param[in] y_body_acc Desired y-acceleration.
     * @param[in] thrust Current quadrotor thrust in Newtons.
     * @return Struct with the desired roll and pitch, in degrees.
     */
    RollPitchCommand mapAccelToAngle(double x_body_acc, double y_body_acc,
                          double thrust);

    PIDControllerBase x_controller_base_;
    PIDControllerBase y_controller_base_;
    
    // Maximum roll/pitch angles to command, in radians.
    static const double DEFAULT_MAX_ROLL_PITCH_ANGLE;
    double rp_max_angle_deg_;


    // Gain is in: m/s/s / m
    static const double DEFAULT_TRANSLATION_KP;
    static const double DEFAULT_TRANSLATION_KI;
    static const double DEFAULT_TRANSLATION_KD;

    static const double DEFAULT_INTEGRATOR_LIMIT;
    double integrator_limit_;

    static const double DEFAULT_MAX_TRANSLATION_ERROR;
    double max_translation_error_;
    static const double DEFAULT_MAX_TRANSLATION_RATE_ERROR;
    double max_translation_rate_error_;

    static const std::string TRANSLATION_CONTROLLER_NAME;

    // Mass of the quadrotor system, in kg.
    double mass_;
};


#endif // QR_OUTERLOOP_PID_INCLUDE_QR_OUTERLOOP_PID_TRANSLATION_CONTROLLER_H_

