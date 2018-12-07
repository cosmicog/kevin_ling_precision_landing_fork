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
 * File: pelican_ips_estimator_node.cc
 * Desc: Entry point for quadrotor state estimator that combines the onbaord
 *   state estimates of the Asctec Pelican with indoor positioning system
 *   measurements.
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
#include <geometry_msgs/Pose.h>
#include <asctec_hl_comm/mav_imu.h>

#include <wave_msgs/QRStateEstimate.h>

#include <wave_pelican/pelican_utils.h>
#include <wave_utils/wave_math.h>

#include <qr_ips_estimator/ips_differentiator.h>


const double DEFAULT_CUTOFF_FREQUENCY = 3;
const double DEFAULT_MAX_RATE = 50;

/**
 * Pelican quadrotor 12 state estimator. Position, linear velocity, and yaw
 * estimates are obtained from the indoor positioning system (IPS) measurements.
 * Roll, pitch, and all three angular rates are taken from sensors onboard the
 * Asctec Pelican.
 */
class PelicanIPSEstimator {
  public:
    /**
     * Constructor.
     *
     * @param nh ROS NodeHandle to register subscribers/publishes.
     * @param max_rate The maximum estimate output rate, in Hz.
     * @param cutff The cutoff frequency for the low pass filters on position
     *    velocity estimates.
     */
    PelicanIPSEstimator(ros::NodeHandle nh, double max_rate, double cutoff);
    ~PelicanIPSEstimator() { }

    void ipsSubscriberCallback(const geometry_msgs::Pose& msg);

    void pelicanSensorSubscriberCallback(const asctec_hl_comm::mav_imu& msg);

  private:
    ros::Subscriber ips_subscriber_;
    ros::Subscriber pelican_sensor_subscriber_;
    ros::Publisher qr_state_estimate_publisher_;

    // Counts of incoming messages, for the watchdog.
    double ips_rx_count_;
    double pelican_sensor_rx_count_;

    IPSDifferentiator estimator_base_;

    asctec_hl_comm::mav_imu last_pelican_sensor_msg_;

    // Minimum period between published estimates to enforce the max rate.
    double min_period_; // 1 / max_rate, in seconds
};

// TODO: Add watchdog to make sure messages are coming in at reasonable rates.

PelicanIPSEstimator::PelicanIPSEstimator(ros::NodeHandle nh,
                                         double max_rate,
                                         double cutoff) :
                                         estimator_base_(cutoff),
                                         ips_rx_count_(0),
                                         pelican_sensor_rx_count_(0) {

  ips_subscriber_ = nh.subscribe("pelican/pose",
                                 1,
                                 &PelicanIPSEstimator::ipsSubscriberCallback,
                                 this);

  pelican_sensor_subscriber_ = nh.subscribe("fcu/imu_custom",
                                  1,
                                  &PelicanIPSEstimator::pelicanSensorSubscriberCallback,
                                  this);

  qr_state_estimate_publisher_ = nh.advertise<wave_msgs::QRStateEstimate>(
                                    "wave/qr_inertial_estimate", 1);

  min_period_ = 1.0 / max_rate;
}

void PelicanIPSEstimator::ipsSubscriberCallback(
    const geometry_msgs::Pose& msg) {

  ips_rx_count_++;

  estimator_base_.processIPSMeasurements(msg);

  if (!pelican_sensor_rx_count_ || !ips_rx_count_) {
    return;
  }

  // Enforce the max estimate publishing rate.
  static ros::Time last_run_time(0);
  ros::Time current_time = ros::Time::now();
  if ((current_time - last_run_time).toSec() < min_period_) {
    return;
  }
  last_run_time = current_time;

  Eigen::Vector4d q_ips(msg.orientation.x, msg.orientation.y,
                        msg.orientation.z, msg.orientation.w);
  Eigen::Vector3d ips_rpy = wave_utils::quaternionToEuler321(
                                q_ips, 0.001 /*quaternion tolerance*/);
  Eigen::Vector4d q_imu(last_pelican_sensor_msg_.orientation.x,
                        last_pelican_sensor_msg_.orientation.y,
                        last_pelican_sensor_msg_.orientation.z,
                        last_pelican_sensor_msg_.orientation.w);
  Eigen::Vector3d imu_rpy = wave_utils::quaternionToEuler321(
                                q_imu, 0.001 /*quaternion tolerance*/);

  wave_msgs::QRStateEstimate qr_state;
  qr_state.header.stamp = ros::Time::now();
  qr_state.type = wave_msgs::QRStateEstimate::EST_12_STATE;
  qr_state.x = msg.position.x;
  qr_state.y = msg.position.y;
  qr_state.z = msg.position.z;
  qr_state.dx = estimator_base_.get_x_velocity();
  qr_state.dy = estimator_base_.get_y_velocity();
  qr_state.dz = estimator_base_.get_z_velocity();
  qr_state.roll = imu_rpy(0);
  qr_state.pitch = imu_rpy(1);
  qr_state.yaw = ips_rpy(2);
  qr_state.droll = last_pelican_sensor_msg_.angular_velocity.x;
  qr_state.dpitch = last_pelican_sensor_msg_.angular_velocity.y;
  qr_state.dyaw = last_pelican_sensor_msg_.angular_velocity.z;

  qr_state_estimate_publisher_.publish(qr_state);
}

void PelicanIPSEstimator::pelicanSensorSubscriberCallback(
    const asctec_hl_comm::mav_imu& msg) {

  pelican_sensor_rx_count_++;
  last_pelican_sensor_msg_ = msg;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "pelican_ips_estimator_node");
  ros::NodeHandle nh;
  ros::NodeHandle nh_("~");

  double max_rate;
  nh.param<double>("wave/max_estimate_rate", max_rate, DEFAULT_MAX_RATE);

  double cutoff;
  nh_.param<double>("lpf_cutoff_frequency", cutoff, DEFAULT_CUTOFF_FREQUENCY);

  PelicanIPSEstimator estimator(nh, max_rate, cutoff);
  ros::spin();
}

