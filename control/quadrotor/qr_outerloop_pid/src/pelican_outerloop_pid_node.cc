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
 * File: pelican_outerloop_pid_node.cc
 * Desc: Program entry point and wrapper to use the outerloop_pid_controller
 *       with the Asctec Pelican platform.
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

#include <ros/ros.h>

#include <asctec_hl_comm/mav_ctrl.h>
#include <wave_pelican/pelican_utils.h>

#include <qr_outerloop_pid/outerloop_pid_controller.h>

OuterloopPIDController* controller_base_ptr_;

/**
 * Implements the ControlMessagePublisher interface to be used to publish
 * control messages to the asctec_mav_framework.
 */
class PelicanControlMessagePublisher : public ControlMessagePublisher
{
  public:
    PelicanControlMessagePublisher(ros::NodeHandle nh);
    ~PelicanControlMessagePublisher() {/*Empty Destructor*/};

    void publishControlMessageCallback(double thrust, double roll,
                                       double pitch, double yaw_rate);
  private:
    ros::Publisher pelican_pub;
    ros::NodeHandle nh_;
};

PelicanControlMessagePublisher::PelicanControlMessagePublisher(
    ros::NodeHandle nh) : nh_(nh) {
  pelican_pub = nh.advertise<asctec_hl_comm::mav_ctrl>("fcu/control", 1);
}


void PelicanControlMessagePublisher::publishControlMessageCallback(
    double thrust, double roll, double pitch, double yaw_rate) {

  PelicanUtils& pelican_utils_ = PelicanUtils::getInstance();

  asctec_hl_comm::mav_ctrl ctrl_msg;
  ctrl_msg.header.stamp = ros::Time::now();
  ctrl_msg.type = asctec_hl_comm::mav_ctrl::acceleration;
  ctrl_msg.x = PelicanUtils::angleToPelicanCommand(pitch);
  ctrl_msg.y = PelicanUtils::angleToPelicanCommand(roll);
  ctrl_msg.z = pelican_utils_.collectiveForceToThrustCommand(thrust);
  ctrl_msg.yaw = yaw_rate;
  pelican_pub.publish(ctrl_msg);
}

void autonomousModeSubscriberCallback(const asctec_hl_comm::mav_status& msg) {
  bool autonomous_mode = PelicanUtils::checkAutonomousMode(msg);
  if (autonomous_mode) {
    controller_base_ptr_->startController();
  } else {
    controller_base_ptr_->stopController();
  }
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "pelican_outerloop_pid");
  ros::NodeHandle nh;

  PelicanUtils& pelican_utils = PelicanUtils::getInstance();

  PelicanControlMessagePublisher pelican_publisher(nh);
  OuterloopPIDController outerloop_controller_base(nh,
                                                   pelican_utils.get_mass_kg(),
                                                   pelican_publisher);
  controller_base_ptr_ = &outerloop_controller_base;
  
  ros::Subscriber autonomous_mode_subscriber = nh.subscribe("fcu/status", 1,
      autonomousModeSubscriberCallback);

  ros::spin();

  return 0;
}

