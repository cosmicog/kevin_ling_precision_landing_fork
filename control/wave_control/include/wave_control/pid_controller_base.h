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
 * File: pid_controller_base.h
 * Desc: Prototypes for the PIDControllerBase class.
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

#ifndef WAVE_CONTROL_INCLUDE_WAVE_CONTROL_PID_CONTROLLER_BASE_H_ 
#define WAVE_CONTROL_INCLUDE_WAVE_CONTROL_PID_CONTROLLER_BASE_H_ 

#include <string>

#include <ros/ros.h>

/**
 * A basic PID controller implementation with several features:
 *  - The gains for the controller can be changed on the fly via the ROS
 *    Parameter Server.
 *  - Optional logging messages that show the individual contributions of each
 *    controller gain.
 *  - Optional output saturation.
 *  - Optional integrator saturation.
 *  - Integrators can be set manually.
 *  - Configurable timeout to automatically reset the integrators if the time
 *    between control computations is too long (usually occurs when first
 *    running the controller after a rest). This prevents spikes in the integral
 *    term. Defaults to 1 second timeout.
 *  - Error handling for proportional and derivative errors that are NaN, only
 *    when the gains that are applied to the respective error is non-zero.
 *
 * Uses the following ROS parameters:
 *    - [NodeHandle path]/enable_pid_logging - <bool> True to enable logging
 *    - [NodeHandle path]/[controller_name]/Kp - <double> Proportional gain
 *    - [NodeHandle path]/[controller_name]/Ki - <double> Integral gain
 *    - [NodeHandle path]/[controller_name]/Kd - <double> Derivative gain
 *
 * When logging is enabled, log messages are published to:
 *    [NodeHandle path]/[controller_name]/pid_log
 *
 * If the ROS parameters for the gains are not set, the default gains passed
 * in the constructor are used. Gains can still be changed via the ROS
 * parameters even if the default gains are used.
 */
class PIDControllerBase
{
  public:
    /**
     * Constructor for a PIDControllerBase object.
     *
     * @param[in] nh Public ROS NodeHandle for the current node. Used to
     *    register a publisher for controller output logging and for getting
     *    ros parameters.
     * @param[in] default_kp The default proportional gain.
     * @param[in] default_ki The default integral gain.
     * @param[in] default_kd The default derivative gain.
     * @param[in] controller_name The name of the controller, used for the
     *    log message's topic and to find the gain dictionary ROS parameter.
     */
    PIDControllerBase(ros::NodeHandle nh, double default_kp, double default_ki,
        double default_kd, std::string controller_name);

    /**
     * PIDControllerBase destructor.
     */
    ~PIDControllerBase() { /* Empty destructor */};

    /**
     * Sets the upper and lower saturator limits of the controller output. The
     * larger limit is automatically set as the upper bound. Output saturation
     * is disabled by default until the limits are set with this function.
     *
     * @param[in] lim1 The first output saturation limit.
     * @param[in] lim2 The second output saturation limit.
     */
    void setOutputSaturationLimits(double lim1, double lim2);

    /**
     * Sets the upper and lower saturator limits on the integrator value. The
     * larger limit is automatically set as the upper bound. Integrator
     * saturation is disabled by default until the limits are set with this
     * function.
     *
     * The proportional error is scaled by the integral gain prior to
     * integration, so the current integrator value is the integral control's
     * contribution to the output control signal.
     *
     * @param[in] lim1 The first integrator saturation limit.
     * @param[in] lim2 The second integrator saturation limit.
     */
    void setIntegratorLimits(double lim1, double lim2);

    /**
     * Sets the controller's timeout duration. If the time between run() calls
     * exceeds the timeout duration then the integrator value will be reset to
     * zero. Defaults to 1 second.
     *
     * @param[in] duration The timeout duration. Duration must be >= 0.
     */
    void setTimeoutDuration(double duration);

    /**
     * Sets the integrator value to 0.
     */
    void resetIntegrator();

    /**
     * Sets the integrator to the specified value. The proportional error is
     * scaled by the integral gain prior to integration, so the current
     * integrator value is the integral control's contribution to the output
     * control signal.
     *
     * @param[in] integrator The new integrator value.
     */
    void setIntegrator(double integrator);

    /**
     * Runs the PID controller with the specified error values. The time
     * between calls to this method is automatically calculated in order to
     * update the integrator.
     *
     * @param[in] proportional_error The proportional error of the system.
     * @param[in] derivative_error The derivative error of the system.
     * @return The control signal output.
     */
    double run(double proportional_error, double derivative_error);

  
    /**
     * Accessor for the integrator value.
     */
    double getIntegrator() {return integrator_;}

  private:
    // Controller gains (proportional, integral, and dervative, respectively).
    double kp_;
    double ki_;
    double kd_;

    // Name of the controller. Also the name of the ROS parameter with the 
    // corresponding gain dictionary.
    std::string controller_name_;

    // Controller integrator.
    double integrator_;

    // Timeout duration. If calls to run() take longer than timeout_duration_,
    // the integrator will be reset.
    double timeout_duration_;

    // Output saturation limits
    double output_lim1_;
    double output_lim2_;
    bool output_saturator_enabled_;

    // Integrator saturation limits
    double integrator_lim1_;
    double integrator_lim2_;
    bool integrator_saturator_enabled_;
    
    // Publisher for PID data
    ros::Publisher controller_log_pub_;

    // Publish log messages if logging_enabled_
    bool logging_enabled_;

    // Internal flag to make sure a freshly set integrator (from a 
    // setIntegrator() call) is not immediately reset due to a timeout.
    bool integrator_freshly_set_;

    // Last time run() was called. Used to calculate if the timeout_duration_
    // has been exceeded and to calculate elapsed time for error integration.
    ros::Time last_run_time_;

    // NodeHandle for fetching ros params.
    ros::NodeHandle nh_;
};

#endif // WAVE_CONTROL_INCLUDE_WAVE_CONTROL_PID_CONTROLLER_BASE_H_ 

