
#include <ros/ros.h>
#include <math.h>
#include "asctec_msgs/GPSData.h"
#include "geometry_msgs/PointStamped.h"



#define DEG2RAD M_PI/180.0

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
  geometry_msgs::PointStamped gps_pos_enu;

  
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

  double slat = sin(wlat*DEG2RAD);
  double clat = cos(wlat*DEG2RAD);
  double r_n = A_EARTH/sqrt(1 - NAV_E2*slat*slat);

  ecef_coords->x = (r_n + walt)*clat*cos(wlon*DEG2RAD);
  ecef_coords->y = (r_n + walt)*clat*sin(wlon*DEG2RAD);
  ecef_coords->z = (r_n*(1 - NAV_E2) + walt)*slat;
}


void convert_to_enu(meters_loc *ecef_coords, meters_loc *enu_coords)
{

  double xdiff = ecef_coords->x - gps.starting_ecef_loc.x;
  double ydiff = ecef_coords->y - gps.starting_ecef_loc.y;
  double zdiff = ecef_coords->z - gps.starting_ecef_loc.z;
  
  double lambda = DEG2RAD*gps.starting_wgs_loc.lon;
  double phi = DEG2RAD*gps.starting_wgs_loc.lat;

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
    gps.starting_wgs_loc.lat = msg.latitude / pow(10,7);
    gps.starting_wgs_loc.lon = msg.longitude / pow(10,7);
    gps.starting_wgs_loc.alt = msg.height / 1000.0;
    convert_to_ecef(&gps.starting_wgs_loc, &gps.starting_ecef_loc);
//    convert_to_enu(&gps.starting_ecef_loc, &gps.gps_pos_enu);
    gps.init_meas = false;
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
    
    gps.gps_pos_enu.header = msg.header;
    gps.gps_pos_enu.header.frame_id = "/wave_inertial";
    gps.gps_pos_enu.point.x = current_enu.x;
    gps.gps_pos_enu.point.y = current_enu.y;
    gps.gps_pos_enu.point.z = current_enu.z;
    gps.gps_pub.publish(gps.gps_pos_enu);
  }
}

void init_gps_messaging(ros::NodeHandle nh)
{
  gps.gps_sub = nh.subscribe("/asctec/GPS_DATA", 1, pelican_gps_cb);

  gps.gps_pub = nh.advertise<geometry_msgs::PointStamped> ("/wave/gps_enu", 1);

  gps.init_meas = true;
}


