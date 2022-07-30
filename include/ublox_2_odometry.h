#ifndef UBOX_2_ODOMETRY_H
#define UBLOX_2_ODOMETRY_H

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <geometry_msgs/TwistWithCovariance.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "ublox_msgs/NavRELPOSNED9.h"
#include <sensor_msgs/NavSatFix.h>
#include "robot_localization/navsat_conversions.h"
#include <string>

class ublox_2_odometry
{

public:
    ublox_2_odometry(std::string map_frame, std::string base_frame);

private:
    void fixCallback(const sensor_msgs::NavSatFix::ConstPtr &msg);
    void navrelposnedCallback(const ublox_msgs::NavRELPOSNED9::ConstPtr &msg);
    void publishOdometry();
    void handleGPSUpdate(tf2::Vector3 gps_pos, double gps_accuracy_m);
    double getGPSX(ublox_msgs::NavRELPOSNED9 &msg);
    double getGPSY(ublox_msgs::NavRELPOSNED9 &msg);
    double getGPSZ(ublox_msgs::NavRELPOSNED9 &msg);

    // Datum coordinates as reference if using lat long coordinates.
    double datumN, datumE, datumLat, datumLng;
    std::string datumZone;

    tf2::Vector3 last_gps_pos;
    double last_gps_acc_m;
    ros::Time last_gps_odometry_time;
    int gps_outlier_count = 0;
    int valid_gps_samples = 0;
    bool gpsOdometryValid = false;
    bool gpsEnabled = true;
    geometry_msgs::Quaternion orientation_result;

    std::string _frame_map = "map";
    std::string _frame_base_link = "base_link";

    bool firstData = true;

    // inouts here
    double dt = 1.0;
    // outputs here
    double x = 0, y = 0, vx = 0.0, r = 0.0, vy = 0.0, vr = 0.0;

    ros::NodeHandle nh;
    ros::Publisher _odom_pub;
    ros::Subscriber _fix_sub;
    ros::Subscriber _navlrelposned_sub;
};

#endif