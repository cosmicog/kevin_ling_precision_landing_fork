/**
 * Unit tests for functions in the wave_utils package.
 */

#include <wave_utils/wave_math.h>
#include <wave_utils/wave_string.h>

#include <assert.h>
#include <math.h>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <Eigen/LU>

#include <ros/ros.h>
#include <tf/tf.h>

void test_sgn()
{
  ROS_INFO("Testing wave_sgn...");
  assert(1 == wave_utils::sgn((double) 15));
  assert(1 == wave_utils::sgn((int) 10));
  assert(0 == wave_utils::sgn(0));
  assert(-1 == wave_utils::sgn((float) -15));
  assert(-1 == wave_utils::sgn((short) - 10));
  ROS_INFO("PASS");
}


void test_saturate()
{
  ROS_INFO("Testing saturate...");
  double dtemp = 11;
  assert(10 == wave_utils::saturate(dtemp, (double)10, (double)-10));

  dtemp = -11;
  assert(-10 == wave_utils::saturate(dtemp, (double)10, (double)-10));
  assert(-10 == wave_utils::saturate(dtemp, (double)-10, (double)10));

  float ftemp = 5;
  assert(5 == wave_utils::saturate(ftemp, (float)10, (float)-10));
  assert(5 == wave_utils::saturate(ftemp, (float)-10, (float)10));
  assert(-10 == wave_utils::saturate(ftemp, (float)-10, (float)-10));

  ftemp = -5;
  assert(-5 == wave_utils::saturate(ftemp, (float)10, (float)-10));

  int itemp = 0;
  assert(0 == wave_utils::saturate(itemp, -10, 10));

  ROS_INFO("PASS");
}


void test_unwrap()
{
  ROS_INFO("Testing unwrapAngle...");
  double dtemp = 0.5*M_PI;
  assert(0.5*M_PI == wave_utils::unwrapAngle(dtemp));

  dtemp = M_PI;
  
  assert(M_PI == wave_utils::unwrapAngle(dtemp));

  dtemp = -M_PI;
  assert(M_PI == wave_utils::unwrapAngle(dtemp));
  ROS_INFO("PASS");

  dtemp = -1.5*M_PI;
  assert(0.5*M_PI == wave_utils::unwrapAngle(dtemp));

  dtemp = M_PI;
  assert(M_PI == wave_utils::unwrapAngle(dtemp));

  float ftemp = 0.75*M_PI;
  assert((float)(0.75*M_PI) == wave_utils::unwrapAngle(ftemp));
}


void test_quaternionToEuler321() {
  ROS_INFO("Testing quaternionToEuler321");

  // Quaternion for R:25, P:50, Y:30, generated with SpinCalc.
  tf::Quaternion q(0.140654953821611, 0.449311678406238,
                   0.082688013720106, 0.878349527238578);
  Eigen::Vector4d quaternion(q.getX(), q.getY(), q.getZ(), q.getW());
  double x = quaternion(0);
  double y = quaternion(1);
  double z = quaternion(2);
  double w = quaternion(3);
  
  ROS_INFO("Quaternion before call: %f, %f, %f, %f", x, y, z, w);

  Eigen::Vector3d rpy = wave_utils::quaternionToEuler321(quaternion, 0.0001);
  Eigen::Vector3d rpy2 = wave_utils::tfQuaternionToEuler321(q, 0.0001);

  ROS_INFO("roll: %f, pitch: %f, yaw: %f", wave_utils::radToDeg(rpy(0)), wave_utils::radToDeg(rpy(1)), wave_utils::radToDeg(rpy(2)));
  ROS_INFO("roll: %f, pitch: %f, yaw: %f", wave_utils::radToDeg(rpy2(0)), wave_utils::radToDeg(rpy2(1)), wave_utils::radToDeg(rpy2(2)));

  // Test an unnormalized vector
  Eigen::Vector4d quaternion2(2*q.getX(), 2*q.getY(), 2*q.getZ(), 2*q.getW());
  Eigen::Vector3d rpy3 = wave_utils::quaternionToEuler321(quaternion2, 0.0001);
  ROS_INFO("roll: %f, pitch: %f, yaw: %f", wave_utils::radToDeg(rpy3(0)), wave_utils::radToDeg(rpy3(1)), wave_utils::radToDeg(rpy3(2)));
}

void test_split() {
  ROS_INFO("Testing split().");
  std::string test_string("This/is/a//test/string");

  std::vector<std::string> splitted = wave_utils::split(test_string, '/');
  assert(splitted.size() == 6);
  ROS_INFO("%s %s", splitted[0].c_str(), splitted[1].c_str());
}

void test_norm2() {
  ROS_INFO("Testing norm2().");
  assert(5.0 == wave_utils::norm2(3.0, 4.0));
  assert(13.0 == wave_utils::norm2(12, 5));
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "wave_utils_test");
  ros::NodeHandle nh;
 
  ROS_INFO("Starting wave_utils unit tests");

  test_sgn();

  ROS_INFO("Testing createEulerRot NOT tested.");

  test_saturate();

  test_unwrap();

  test_quaternionToEuler321();

  test_norm2();

  test_split();
  ROS_INFO("All unit tests passed");

  return 0;
}

