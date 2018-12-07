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
 * File: outerloop_pid_controller.h
 * Desc: Defines prototypes for OuterloopPIDController class and
 *       the ControlMessagePublisher interface.
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

#ifndef QR_OUTERLOOP_PID_INCLUDE_QR_OUTERLOOP_PID_OUTERLOOP_PID_CONTROLLER_H_
#define QR_OUTERLOOP_PID_INCLUDE_QR_OUTERLOOP_PID_OUTERLOOP_PID_CONTROLLER_H_

#include <string>

#include <ros/ros.h>

#include <qr_outerloop_pid/SetTranslationControllerStatus.h>
#include <wave_msgs/QRControl.h>
#include <wave_msgs/QRStateEstimate.h>

#include <qr_outerloop_pid/height_controller.h>
#include <qr_outerloop_pid/translation_controller.h>
#include <qr_outerloop_pid/yaw_controller.h>

/**
 * Interface to provide a callback function to the OuterloopPIDController.
 */
class ControlMessagePublisher
{
  public:
    virtual ~ControlMessagePublisher() {/*Empty Destructor*/}

    /**
     * Callback message called by the OuterloopPIDController to publish
     * commanded values to the robot. The final unit conversions from SI to
     * platform specific units as well as any required coordinate frame
     * transforms should be done in this callback. Finally, the callback should
     * publish a ROS message with the desired commands to the appropriate topic.
     */
    virtual void publishControlMessageCallback(double thrust,
                                               double roll,
                                               double pitch,
                                               double yaw_rate) = 0;
};


/**
 * Controller that performs position level control for a quadrotor. This class
 * subscribes to ROS topics to receive the commands, inertial state estimates,
 * and relative state estimates as the inputs. The outputs are the thrust,
 * roll, pitch, and yaw_rate commands to be sent to the platform specific low
 * level controller. These outputs are passed to the ControlMessagePublisher
 * that was specified in the constructor of this class.
 *
 * Supports the following  modes defined in the wave_msgs::QRControl message:
 *    - CMD_INERTIAL_POSITION
 *    - CMD_INERTIAL_TRAJECTORY
 *    - CMD_RELATIVE_POSITION
 *
 * See pelican_outerloop_pid_node.cc for a usage example.
 */
class OuterloopPIDController
{
  public:
    /**
     * Constructor. Subscribes to the appropriate topics for the QRStateEstimate
     * and QRControl messages.
     *
     * @param[in] nh Public NodeHandle of current node.
     * @param[in] mass Mass of the quadrotor system.
     * @param[in] parent
     */
    OuterloopPIDController(ros::NodeHandle nh, double mass,
                           ControlMessagePublisher& message_publisher);

    ~OuterloopPIDController() {/* Empty Destructor */};

    /**
     * Control messages will not be published to the low-level processor when
     * the controller is stopped. Message publications will resume when
     * startController() is called. By default, the controller is stopped.
     */
    void stopController();

    /**
     * Enables the controller. If the controller was previously disabled, the
     * integrators will be reset.
     */
    void startController();

    /**
     * ROS callback function for the inertial state estimate message.
     */
    void inertialEstimateSubscriberCallback(
        const wave_msgs::QRStateEstimate& msg);

    /**
     * ROS callback function for the relative state estimate message.
     */
    void relativeEstimateSubscriberCallback(
        const wave_msgs::QRStateEstimate& msg);

    /**
     * ROS callback function for the command messages.
     */
    void commandSubscriberCallback(const wave_msgs::QRControl& msg);

    /**
     * ROS service callback for enabling/disabling the translation controller.
     */
    bool translationControllerStatusServiceCallback(
        qr_outerloop_pid::SetTranslationControllerStatus::Request& request,
        qr_outerloop_pid::SetTranslationControllerStatus::Response& response);

    unsigned int get_estimate_rx_count() {return inertial_estimate_rx_count_;}
    unsigned int get_command_rx_count() {return command_rx_count_;}
  private:
    /**
     * Updates the controllers and publishes the output. It will automatically
     * throttle calls to ensure that control is performed at a desired
     * maximum control rate. This function will return immediately if the
     * controller is not enabled.
     */
    void run();

    // Interface that holds the callback function to publish desired commands
    // to the platform specific innerloop controller.
    ControlMessagePublisher& message_publisher_;

    HeightController height_controller_;
    YawController yaw_controller_;
    TranslationController translation_controller_;

    ros::Subscriber inertial_estimate_subscriber_;
    ros::Subscriber relative_estimate_subscriber_;
    ros::Subscriber command_subscriber_;

    // Service to start/stop translation controller
    ros::ServiceServer translation_controller_service_;

    // Stored values of the last received messages.
    wave_msgs::QRStateEstimate latest_inertial_estimate_;
    wave_msgs::QRStateEstimate latest_relative_estimate_;
    wave_msgs::QRControl latest_command_;

    // Number of received messages. Used to make sure messages are received
    // and checked by the watchdog.
    unsigned int inertial_estimate_rx_count_;
    unsigned int relative_estimate_rx_count_;
    unsigned int command_rx_count_;

    // Calls to run() without at least this much time since the
    // last control update time will be ignored.
    double min_control_period_;
    
    // Default max controller frequency that dictates the min_control_period_.
    static const double DEFAULT_MAX_CONTROL_RATE; // Hz

    // Main switch for all four controllers. Set automatically by the status
    // of the vehicle.
    bool controller_enabled_;

    // Translation control is only run if both translation_control_enabled_ and
    // controller_enabled_ are true. If this is false but controller_enabled_ is
    // true, only the height and yaw controllers will be run. Neutral roll
    // and pitch are commanded when translation control is off.
    bool translation_control_enabled_;
};

#endif // QR_OUTERLOOP_PID_INCLUDE_QR_OUTERLOOP_PID_OUTERLOOP_PID_CONTROLLER_H_

