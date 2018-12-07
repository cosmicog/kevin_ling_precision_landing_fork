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
 * File: wave_math.h
 * Desc: Prototypes for a collection of math related utility functions.
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

#ifndef WAVE_UTILS_INCLUDE_WAVE_UTILS_WAVE_MATH_H_
#define WAVE_UTILS_INCLUDE_WAVE_UTILS_WAVE_MATH_H_

#define _USE_MATH_DEFINES
#include <math.h>

#include <Eigen/Core>
#include <Eigen/LU>

#include <tf/tf.h>

/**
 * Namespace for WAVELab utility functions. 
 */
namespace wave_utils {

// Degrees to radians conversion factor.
const double DEGREES_TO_RADIANS = M_PI/180.0;

// Radians to degrees conversion factor.
const double RADIANS_TO_DEGREES = 180.0/M_PI;

// Magnitude of acceleration due to gravity, in m/s^2.
const double GRAVITY_MSS = 9.81;


/**
 * Creates and returns a rotation matrix from the specified 3-2-1 Euler
 * angles. When applied as a left-multiplication, this rotation matrix
 * will rotate a vector from its original frame into a rotated frame
 * defined by the given angles with respect to the original frame.
 *
 * Order of rotations: yaw, pitch, roll.
 *
 * @param[in] roll Roll Euler angle, in radians.
 * @param[in] pitch Pitch Euler angle, in radians.
 * @param[in] yaw Yaw Euler angle, in radians.
 * @return A 3x3 rotation matrix with the specified rotations.
 */
Eigen::Matrix3d createDCMFromRPY(double roll, double pitch,
                                        double yaw);


/**
 * Performs an Euler rotation on a given two-dimensional vector by a
 * given yaw.
 *
 * @param[in] input Vector to rotate.
 * @param[in] yaw Yaw Euler angle, in radians.
 * @return The resulting (rotated) vector.
 */
Eigen::Vector2d rotateByYawInR2(Eigen::Vector2d input, double yaw);


/**
 * Performs a 3-2-1 Euler rotation on a given three-dimensional vector.
 *
 * Order of rotations: yaw, pitch, roll.
 *
 * @param[in] input Vector to rotate.
 * @param[in] roll Roll Euler angle, in radians.
 * @param[in] pitch Pitch Euler angle, in radians.
 * @param[in] yaw Yaw Euler angle, in radians.
 * @return The resulting (rotated) vector.
 */
Eigen::Vector3d rotateInR3(Eigen::Vector3d input,
                                  double roll,
                                  double pitch,
                                  double yaw);


/**
 * Convert a quaternion to 3-2-1 euler angles.
 *
 * @param[in] q Quaternion in the ROS tf::Quaternion form.
 * @param[in] tolerance Quaternions with magnitudes that deviate from unity by
 *    more than this tolerance will be normalized.
 * @return A vector with the equivalent [roll, pitch, yaw].
 */
Eigen::Vector3d tfQuaternionToEuler321(tf::Quaternion q, double tolerance);


/**
 * Convert a quaternion to 3-2-1 euler angles.
 *
 * @param[in] q Quaternion vector in the order [x, y, z, w].
 * @param[in] tolerance Quaternions with magnitudes that deviate from unity by
 *    more than this tolerance will be normalized.

 * @return The equivalent rotation in roll, pitch, and yaw, in that order.
 */
Eigen::Vector3d quaternionToEuler321(Eigen::Vector4d q, double tolerance);


/**
 * Signum function.
 *
 * @param[in] val A number to check the sign of.
 * @return 1 if val is positive, -1 if val is negative, 0 otherwise.
 */
template <typename T> int sgn(T val) {
  return (T(0) < val) - (val < T(0));
}


/**
 * Constrains a number between the specified limits. The upper and lower
 * limits are automatically determined by comparing the inputs lim1 and
 * lim2.
 * 
 * All inputs must be of the same type. The numbers are converted to type
 * double before saturating, and then converted back to the input type.
 *
 * @param[in] val The number to saturate.
 * @lim1[in] The first saturation limit.
 * @lim2[in] The second saturation limit.
 * @return The saturated val input.
 */
template <typename T> T saturate(T val, T lim1, T lim2) {
  // Note: Must cast to double so that everything works with fmax and fmin.

  double upper_limit;
  double lower_limit;

  // If they are equal then it doesn't matter which limit is what.
  if (lim1 >= lim2) {
    upper_limit = static_cast<double>(lim1);
    lower_limit = static_cast<double>(lim2);
  } else {
    lower_limit = static_cast<double>(lim1);
    upper_limit = static_cast<double>(lim2);
  }

  double temp = static_cast<double>(val);
  temp = fmax(lower_limit, temp);
  temp = fmin(upper_limit, temp);
  return static_cast<T>(temp);
}


/**
 * Performs an angle wrap around correction to bound an angle in the range
 * of (-pi, pi].
 *
 * @param[in|out] val Angle to correct for wrap around, in radians.
 */
template <typename T> T unwrapAngle(T val) {
  double temp = static_cast<double>(val);
  if (temp <= -M_PI) {
    temp += 2*M_PI;
  } else if (temp > M_PI) {
    temp -= 2*M_PI;
  }

  return static_cast<T>(temp);
}


/**
 * Converts an angle in degrees to radians.
 *
 * @param[in] deg The angle in degrees to convert.
 * @return The angle converted to radians, in the type of the passed in
 *    value.
 */
template <typename T> T degToRad(T deg) {
  double temp = static_cast<double>(deg);
  temp *= DEGREES_TO_RADIANS;
  return static_cast<T>(temp);
}


/**
 * Converts an angle in radians to degrees.
 *
 * @param[in] rad The angle in radians to convert.
 * @return The angle converted to degrees, in the type of the passed in
 *    value.
 */
template <typename T> T radToDeg(T rad) {
  double temp = static_cast<double>(rad);
  temp *= RADIANS_TO_DEGREES;
  return static_cast<T>(temp);
}


template <typename T> double norm2(T x, T y) {
  double dim1 = fabs(static_cast<double>(x));
  double dim2 = fabs(static_cast<double>(y));
  return sqrt(pow(x,2) + pow(y,2));
}


template <typename T> double norm3(T x, T y, T z) {
  double dim1 = fabs(static_cast<double>(x));
  double dim2 = fabs(static_cast<double>(y));
  double dim3 = fabs(static_cast<double>(z));
  return sqrt(pow(x,2) + pow(y,2) + pow(z,2));
}


} // namespace wave_utils

#endif // WAVE_UTILS_INCLUDE_WAVE_UTILS_WAVE_MATH_H_

