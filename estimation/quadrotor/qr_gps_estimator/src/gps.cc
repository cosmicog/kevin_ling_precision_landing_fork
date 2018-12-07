
#include <ros/ros.h>
#include <math.h>
#include <asctec_hl_comm/GpsCustom.h>
#include <wave_utils/wave_math.h>

#include <wave_msgs/ENUWithCovariances.h>

struct wgs_loc
{
  double lat;
  double lon;
  double alt;
};

struct meters_loc
{
  double x;
  double y;
  double z;
};


struct GPS
{
  ros::Subscriber gps_sub;
  ros::Publisher gps_pub;

  
  wgs_loc starting_wgs_loc;
  meters_loc starting_ecef_loc;

  bool init_meas;
} gps;


//GPS Helpers
void convert_to_ecef(wgs_loc *wgs_coords, meters_loc *ecef_coords)
{
  double wlat = wgs_coords->lat;
  double wlon = wgs_coords->lon;
  double walt = wgs_coords->alt;

  //Convert LLA to ECEF
  double A_EARTH = 6378137.0;
  double flattening = 1.0/298.257223563;
  double NAV_E2 = (2.0-flattening)*flattening;

  double slat = sin(wave_utils::degToRad(wlat));
  double clat = cos(wave_utils::degToRad(wlat));
  double r_n = A_EARTH/sqrt(1 - NAV_E2*slat*slat);

  ecef_coords->x = (r_n + walt)*clat*cos(wave_utils::degToRad(wlon));
  ecef_coords->y = (r_n + walt)*clat*sin(wave_utils::degToRad(wlon));
  ecef_coords->z = (r_n*(1 - NAV_E2) + walt)*slat;
}


void convert_to_enu(meters_loc *ecef_coords, meters_loc *enu_coords)
{

  double xdiff = ecef_coords->x - gps.starting_ecef_loc.x;
  double ydiff = ecef_coords->y - gps.starting_ecef_loc.y;
  double zdiff = ecef_coords->z - gps.starting_ecef_loc.z;
  
  double lambda = wave_utils::degToRad(gps.starting_wgs_loc.lon);
  double phi = wave_utils::degToRad(gps.starting_wgs_loc.lat);

  enu_coords->x = -1.0*sin(lambda)*xdiff + cos(lambda)*ydiff;
  enu_coords->y = -1.0*sin(phi)*cos(lambda)*xdiff - sin(phi)*sin(lambda)*ydiff + cos(phi)*zdiff;
  enu_coords->z = cos(phi)*cos(lambda)*xdiff + cos(phi)*sin(lambda)*ydiff + sin(phi)*zdiff;
}


void pelican_gps_cb(const asctec_msgs::GPSData& msg)
{
  if (! (msg.status & 0x01) )
  {
    return;
  }

  if(gps.init_meas)
  {
    // Wait for 5 consecutive messages that are in a 3 m radius circle. Then use the
    // first of the 5 as the origin for the ENU frame.
    ROS_INFO_THROTTLE(60, "GPS ENU: Please wait while GPS initializes.");

    static int num_good_measurements = 0;
    static meters_loc init_measurements_ecef[5];

    wgs_loc current_wgs;
    meters_loc current_ecef;

    current_wgs.lat = msg.latitude / pow(10,7);
    current_wgs.lon = msg.longitude / pow(10,7);
    current_wgs.alt = msg.height / 1000.0;
    convert_to_ecef(&current_wgs, &init_measurements_ecef[num_good_measurements]);

    if (0 == num_good_measurements) {
      gps.starting_ecef_loc = init_measurements_ecef[num_good_measurements];
      num_good_measurements++;
    } else {
      meters_loc current_enu;
      convert_to_enu(&init_measurements_ecef[num_good_measurements], &current_enu);

      if (3 < wave_utils::norm2(current_enu.x, current_enu.y)) {
        num_good_measurements = 0;
      } else {
        num_good_measurements++;

        if (5 == num_good_measurements) {
          gps.init_meas = false;
          ROS_INFO("GPS ENU: Initialization complete.");
        }
      }
    }
  }
  else
  {
    wgs_loc current_wgs;
    meters_loc current_ecef;
    meters_loc current_enu;

    current_wgs.lat = msg.latitude / pow(10,7);
    current_wgs.lon = msg.longitude / pow(10,7);
    current_wgs.alt = msg.height / 1000.0;
    convert_to_ecef(&current_wgs, &current_ecef);
    convert_to_enu(&current_ecef, &current_enu);
    
    wave_msgs::ENUWithCovariances enu_msg;
    enu_msg.header = msg.header;
    enu_msg.header.frame_id = "/wave_inertial";
    enu_msg.x = current_enu.x;
    enu_msg.y = current_enu.y;
    enu_msg.z = current_enu.z;
    enu_msg.position_covariance = msg.position_covariance;
    enu_msg.velocity_x = msg.velocity_x;
    enu_msg.velocity_y = msg.velocity_y;
    enu_msg.velocity_covariance = msg.velocity_covariance;
    gps.gps_pub.publish(enu_msg);
  }
}

void init_gps_messaging(ros::NodeHandle nh)
{
  gps.gps_sub = nh.subscribe("/fcu/gps", 1, pelican_gps_cb);

  gps.gps_pub = nh.advertise<wave_msgs::ENUWithCovariances> ("/wave/gps_enu", 1);

  gps.init_meas = true;
}


