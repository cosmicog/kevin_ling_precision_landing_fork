/**
 * Unit tests for functions in the wave_utils package.
 */

#include <ros/ros.h>
#include <wave_pelican/pelican_utils.h>
#include <assert.h>
#include <math.h>


// TODO: Make these tests better.


int main(int argc, char** argv)
{
  ros::init(argc, argv, "wave_utils_test");
  ros::NodeHandle nh;
 

  PelicanUtils& pelican_utils = PelicanUtils::getInstance();
  

  ROS_INFO("Starting thrust mapping unit tests");
  int commanded_thrust_in = 101;
  int commanded_thrust = static_cast<double>(commanded_thrust_in) * 4095.0 / 200.0;

  ROS_INFO("Cmd Thrust: %d", commanded_thrust);

  double qr_thrust = pelican_utils.thrustCommandToCollectiveForce(commanded_thrust);

  ROS_INFO("Thrust: %f", qr_thrust);


  assert(0 == pelican_utils.thrustCommandToCollectiveForce(0));
  ROS_INFO("Done");

  return 0;
}

