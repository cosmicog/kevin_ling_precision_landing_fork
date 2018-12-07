/**
 * GPS callback. Automatically translates the GPS coordinates into ENU
 * relative to the vehicle's start position and publishes it.
 */

#ifndef __GPS_H__
#define __GPS_H__

// void convert_to_ecef(wgs_loc *wgs_coords, meters_loc *ecef_coords);

// void convert_to_enu(meters_loc *ecef_coords, meters_loc *enu_coords);


// void pelican_gps_cb(const asctec_msgs::GPSDataAdvanced& msg);


void init_gps_messaging(ros::NodeHandle nh);

#endif // __GPS_H__

