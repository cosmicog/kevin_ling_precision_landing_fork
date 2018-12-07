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
 * File: pelican_utils.h
 * Desc: Prototypes for Asctec Pelican related helper functions.
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

#ifndef WAVE_PELICAN_INCLUDE_WAVE_PELICAN_PELICAN_UTILS_H_
#define WAVE_PELICAN_INCLUDE_WAVE_PELICAN_PELICAN_UTILS_H_

#include <ros/ros.h>

#include <asctec_hl_comm/mav_status.h>

/**
 * Singleton class that contains functions for converting between SI units and
 * Pelican specific units. Also contains Pelican specific constants.
 */
class PelicanUtils
{
  public:
    /**
     * Returns the singleton instance of the class. Will create an instance of
     * the class if there is none.
     */
    static PelicanUtils& getInstance();

    /**
     * Converts an Asctec Pelican thrust command to an equivalent force in
     * Newtons, as determined through experimental thrust tests.
     *
     * @param[in] thrust_command The Asctec Pelican thrust command in the range
     *    of 0..1.
     * @return The equivalent thrust in Newtons or 0 when teh commanded_thrust
     *    is outside of the accepted range.
     */
    double thrustCommandToCollectiveForce(double thrust_command);

    /**
     * Converts a thrust in Newtons to the equivalent thrust command in Asctec
     * collective thrust units.
     *
     * @param[in] thrust_newtons Thrust in Newtons.
     * @return The equivalent thrust command in Asctec Pelican collective
     *    thrust units scaled from 0..1.
     */
    float collectiveForceToThrustCommand(double thrust_newtons);

    /**
     * Converts a measured angular velocity from Asctec units to radians/s.
     *
     * @param[in] angular_velocity The measured angular velocity in Asctec
     *    units.
     * @return The angular velcoity in rad/s.
     */
    static inline double asctecAngularVelocityToSI(double angular_velocity) {
      return angular_velocity * ANGVEL_TO_RAD_PER_SECOND_;
    }

    /**
     * Converts an Euler angle estimate from Asctec units to radians.
     *
     * @param[in] angle The estimated angle in Asctec units.
     * @return The angle in radians.
     */
    static inline double asctecAngleToSI(double angle) {
      return angle * ANGLE_TO_RAD_;
    }


    /**
     * Converts a desired yaw rate to the equivalent command in
     * the units and coordinate frame of the asctec_mav_framework.
     *
     * @param[in] yaw_rate Yaw rate to convert, in rad/s.
     * @return The equivalent yaw rate in asctec_mav_framework units.
     */
    static inline float yawRateToPelicanCommand(double yaw_rate) {
      return static_cast<float>(yaw_rate * YAW_RATE_TO_PELICAN_COMMAND);
    }

    /**
     * Converts a desired roll or pitch angle to the equivalent command in
     * the units and frame of the asctec_mav_framework.
     *
     * @param[in] The angle to convert, in radians.
     * @return The equivalent angle for the asctec_mav_framework.
     */
    static inline float angleToPelicanCommand(double angle) {
      return static_cast<float>(angle);
    }

    
    /**
     * Retrieve the mass of the quadrotor system, in kg.
     */
    inline double get_mass_kg() {
      return mass_kg;
    }

    /**
     * Set the mass of the quadrotor system, in kg.
     */
    inline void set_mass_kg(double new_mass) {
      mass_kg = new_mass;
    }

    /**
     * Checks an asctec mav_status message to determine if autonomous mode
     * is enabled. Autonomous mode is enabled if the motors are running and the
     * serial interface is enabled.
     *
     * @param msg An asctec mav_status message to check to determine if the
     *    vehicle is in autonomous mode.
     * @return True if the vehicle is in autonomous mode.
     */
    static bool checkAutonomousMode(const asctec_hl_comm::mav_status& msg);

  private:
    /**
     * Creates a PelicanUtils object. Initializes mass and thrust curve
     * parameters from ROS parameters if they are available.
     */
    PelicanUtils();

    ~PelicanUtils() {/*Empty Desstructor*/};

    PelicanUtils(PelicanUtils const&); // Don't implement, singleton.
    void operator = (PelicanUtils const&); // Don't implement, singleton.


    // Total mass of the quadrotor system, in kg.
    static const double MASS_KG_DEFAULT;
    double mass_kg;

    // Conversion factor from measured angular velocities to rad/s.
    static const double ANGVEL_TO_RAD_PER_SECOND_;

    // Conversion factor from asctec angle units (1000/degree) to radians.
    static const double ANGLE_TO_RAD_;

    // Quadratic thrust curve (thrust command to Newtons) parameters for the
    // Asctec Pelican. Identified Oct 2013.
    //
    // Thrust [Newtons] = thrust_curve_a_*cmd^2 + thrust_curve_b_*cmd +
    //                    thrust_curve_c_;
    //
    // where cmd is the thrust commanded to the pelican on the scale 0..4095.
    double thrust_curve_a_;
    double thrust_curve_b_;
    double thrust_curve_c_;
  
    // Default thrust curve values in case ROS params are not present.
    static const double THRUST_CURVE_A_DEFAULT;
    static const double THRUST_CURVE_B_DEFAULT;
    static const double THRUST_CURVE_C_DEFAULT;
    

    // Factor to convert a desired yaw rate in rad/s to a pelican command.
    static const double YAW_RATE_TO_PELICAN_COMMAND;

    // Max thrust command on the scale of 0..4095. 
    static const double MAX_THRUST_COMMAND;

    // Normalizer to convert from 0..4095 to 0..1 for asctec_mav_framework.
    static const double THRUST_NORMALIZER;
    
    // Never command 0 thrust. That will turn off the motors and they need
    // some time to spin up again.
    static const double MIN_THRUST_COMMAND;

    // Motor status string that indicates the motors are running.
    static const std::string MOTOR_STATUS_RUNNING;
};

#endif // INCLUDE_WAVE_PELICAN_PELICAN_UTILS_H_

