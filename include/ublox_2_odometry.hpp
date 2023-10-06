#ifndef UBOX_2_ODOMETRY_H
#define UBLOX_2_ODOMETRY_H

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_with_covariance.hpp>
#include <geometry_msgs/msg/twist_with_covariance.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "ublox_msgs/msg/nav_relposned9.hpp"
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include "robot_localization/navsat_conversions.hpp"
#include <string>

class ublox_2_odometry : public rclcpp::Node
{

public:
    ublox_2_odometry(std::string map_frame, std::string base_frame);

private:
    void fixCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg);
    void navrelposnedCallback(const ublox_msgs::msg::NavRELPOSNED9::SharedPtr msg);
    void publishOdometry();
    void handleGPSUpdate(tf2::Vector3 gps_pos, double gps_accuracy_m);
    double getGPSX(ublox_msgs::msg::NavRELPOSNED9 &msg);
    double getGPSY(ublox_msgs::msg::NavRELPOSNED9 &msg);
    double getGPSZ(ublox_msgs::msg::NavRELPOSNED9 &msg);

    // Datum coordinates as reference if using lat long coordinates.
    double datumN, datumE, datumLat, datumLng;
    std::string datumZone;

    tf2::Vector3 last_gps_pos;
    double last_gps_acc_m;
    rclcpp::Time last_gps_odometry_time;
    int gps_outlier_count = 0;
    int valid_gps_samples = 0;
    bool gpsOdometryValid = false;
    bool gpsEnabled = true;
    geometry_msgs::msg::Quaternion orientation_result;

    std::string _frame_map = "map";
    std::string _frame_base_link = "base_link";

    bool firstData = true;

    // inouts here
    double dt = 1.0;
    // outputs here
    double x = 0, y = 0, vx = 0.0, r = 0.0, vy = 0.0, vr = 0.0;

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr _odom_pub;
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr _fix_sub;
    rclcpp::Subscription<ublox_msgs::msg::NavRELPOSNED9>::SharedPtr _navlrelposned_sub;
};

#endif