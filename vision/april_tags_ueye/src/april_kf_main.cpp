#include "kf_generic.h"
#include "ros/ros.h"

#include <geometry_msgs/Pose.h>

geometry_msgs::Pose last_msg;
bool new_data = false;
bool reset = false;

ros::Time last_t;
void tag_cb(geometry_msgs::Pose msg)
{
  new_data = true;
  last_msg = msg;

  ros::Time now = ros::Time::now();
  if( (now - last_t).toSec() > 1 )
  {
    reset = true;
  }
}


int main( int argc, char** argv )
{
  //ROS init
  ros::init(argc, argv, "april_test_kf");
  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("tag", 1, tag_cb, ros::TransportHints().tcpNoDelay());
  ros::Publisher pred_pub = n.advertise<geometry_msgs::Pose>("tag_pos_pred", 1);

  KFContext kf;
  kf.mean = Vector4d(0,0,0,0); //[x,y,v_x, v_y]
  kf.pred_mean = Vector4d(0,0,0,0);
  kf.sigma = Matrix4d::Identity();
  kf.pred_sigma = Matrix4d::Identity();
  kf.meas_update_last = false;

  float dt_fixed = 1.0f/15.0f;

  Matrix4d A_matr;
  A_matr << 1, 0, dt_fixed, 0,
            0, 1, 0, dt_fixed,
            0, 0, 1, 0,
            0, 0, 0, 1;
  
  Matrix4d pred_cov;
  pred_cov << 0.5, 0, 0, 0,
              0, 0.5, 0, 0,
              0, 0, 0.9, 0,
              0, 0, 0, 0.9;

  MatrixXd C_matr(2,4);
  C_matr << 1, 0, 0, 0,
            0, 1, 0, 0;

  Matrix2d meas_cov;
  meas_cov << 0.3, 0,
              0, 0.3;

  last_t = ros::Time::now();

  ros::Rate loop_rate(15);
  while (ros::ok())
  {
    kf_prediction_update(kf, A_matr, Vector4d(0,0,0,0), pred_cov);
    
    if (new_data)
    {
      kf_measurement_update(kf, C_matr, Vector2d(last_msg.position.x, last_msg.position.y), meas_cov);

      new_data = false;

      if (reset)
      {
        kf.mean[0] = last_msg.position.x;
        kf.mean[1] = last_msg.position.y;
        kf.pred_mean[0] = last_msg.position.x;
        kf.pred_mean[1] = last_msg.position.y;
      }
    }

    if (kf.meas_update_last)
    {
      last_msg.position.x = kf.mean[0];
      last_msg.position.y = kf.mean[1];
    }
    else
    {
      last_msg.position.x = kf.pred_mean[0];
      last_msg.position.y = kf.pred_mean[1];
    }

    pred_pub.publish(last_msg);

    loop_rate.sleep();
    ros::spinOnce();
  }

  return 0;
}

