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
 * File: wave_math.cpp
 * Desc: Source definitions for a collection of math related utility functions.
 * Note: This library depends on the Eigen3 library so implementing packages
 *       will need to include Eigen3 appropriately.
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

#include <wave_utils/wave_math.h>

namespace wave_utils {

Eigen::Matrix3d createDCMFromRPY(double roll, double pitch,
                                 double yaw) {
  double cos_roll = cos(roll);
  double sin_roll = sin(roll);
  double cos_pitch = cos(pitch);
  double sin_pitch = sin(pitch);
  double cos_yaw = cos(yaw);
  double sin_yaw = sin(yaw);

  Eigen::Matrix3d rot;
  rot(0, 0) = cos_pitch*cos_yaw;
  rot(0, 1) = cos_pitch*sin_yaw;
  rot(0, 2) = -sin_pitch;

  rot(1, 0) = sin_roll*sin_pitch*cos_yaw - cos_roll*sin_yaw;
  rot(1, 1) = sin_roll*sin_pitch*sin_yaw + cos_roll*cos_yaw;
  rot(1, 2) = sin_roll*cos_pitch;

  rot(2, 0) = cos_roll*sin_pitch*cos_yaw + sin_roll*sin_yaw;
  rot(2, 1) = cos_roll*sin_pitch*sin_yaw - sin_roll*cos_yaw;
  rot(2, 2) = cos_roll*cos_pitch;

  return rot;
}


Eigen::Vector2d rotateByYawInR2(Eigen::Vector2d input, double yaw) {
  Eigen::Vector2d output;
  output(0) = cos(yaw)*input(0) + sin(yaw)*input(1);
  output(1) = -sin(yaw)*input(0) + cos(yaw)*input(1);
  return output;
}


Eigen::Vector3d rotateInR3(Eigen::Vector3d input, double roll, double pitch,
                           double yaw) {
  return createDCMFromRPY(roll, pitch, yaw) * input;
}


Eigen::Vector3d tfQuaternionToEuler321(tf::Quaternion q, double tolerance) {
  Eigen::Vector4d quaternion(q.getX(), q.getY(), q.getZ(), q.getW());
  return quaternionToEuler321(quaternion, tolerance);
}


Eigen::Vector3d quaternionToEuler321(Eigen::Vector4d q, double tolerance) {
  double x = q(0);
  double y = q(1);
  double z = q(2);
  double w = q(3);

  double xx = pow(x,2);
  double yy = pow(y,2);
  double zz = pow(z,2);
  double ww = pow(w,2);

  // Normalize if necessary
  double mag2 = xx + yy + zz + ww;
  if (fabs(1.0 - mag2) > tolerance) {
    double mag = sqrt(mag2);
    x /= mag;
    y /= mag;
    z /= mag;
    w /= mag;
    xx = pow(x,2);
    yy = pow(y,2);
    zz = pow(z,2);
    ww = pow(w,2);
  }
  
  double psi = atan2(2*(x*y + z*w), (ww + xx - yy - zz));
  double theta = asin(2*(y*w - x*z));
  double phi = atan2(2*(x*w + z*y), (ww - xx - yy + zz));
  return Eigen::Vector3d(phi, theta, psi);
}


} // namespace wave_utils


