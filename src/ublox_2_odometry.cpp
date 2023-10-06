#include "ublox_2_odometry.hpp"

ublox_2_odometry::ublox_2_odometry(std::string map_frame, std::string base_frame) : Node("ublox2Odometry")
{

    // init ROS stack
    _odom_pub = create_publisher<nav_msgs::msg::Odometry>("ublox_odometry", 10);
    //_fix_sub = create_subscription("ublox/fix", 100, std::bind(&ublox_2_odometry::fixCallback, this,std::placeholders::_1));
    _navlrelposned_sub = create_subscription<ublox_msgs::msg::NavRELPOSNED9>("ublox/navrelposned", 100, std::bind(&ublox_2_odometry::navrelposnedCallback, this, std::placeholders::_1));

    // last_gps_odometry_time(0.0);
    _frame_map = map_frame;
    _frame_base_link = base_frame;

    last_gps_odometry_time = get_clock()->now();

    tf2::Quaternion q_mag;
    q_mag.setRPY(0.0, 0.0, 0.0);
    orientation_result = tf2::toMsg(q_mag);
}

double ublox_2_odometry::getGPSY(ublox_msgs::msg::NavRELPOSNED9 &msg)
{
    return ((double)msg.rel_pos_n * 0.01) + ((double)msg.rel_pos_hpn * 0.0001);
}

double ublox_2_odometry::getGPSX(ublox_msgs::msg::NavRELPOSNED9 &msg)
{
    return ((double)msg.rel_pos_e * 0.01) + ((double)msg.rel_pos_hpe * 0.0001);
}

double ublox_2_odometry::getGPSZ(ublox_msgs::msg::NavRELPOSNED9 &msg)
{
    // For now, we assume a plane
    return 0.0;
}

void ublox_2_odometry::fixCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
{
    double n, e;
    std::string zone;
    robot_localization::navsat_conversions::LLtoUTM(msg->latitude, msg->longitude, n, e, zone);

    tf2::Vector3 gps_pos(
        (e - datumE), (n - datumN), 0.0);

    if (msg->position_covariance_type == 0)
    {
        RCLCPP_INFO(get_logger(), "Dropped GPS Update to position_covariance_type being UNKNOWN");
        return;
    }

    double acc_m = sqrt(msg->position_covariance[0]);

    handleGPSUpdate(gps_pos, acc_m);
}

void ublox_2_odometry::navrelposnedCallback(const ublox_msgs::msg::NavRELPOSNED9::SharedPtr msg)
{
    ublox_msgs::msg::NavRELPOSNED9 gps = *msg;
    double gps_accuracy_m = (double)gps.acc_length / 10000.0f;

    bool gnssFixOK = (msg->flags & 0b0000001);
    bool diffSoln = (msg->flags & 0b0000010) >> 1;
    bool relPosValid = (msg->flags & 0b0000100) >> 2;
    auto carrSoln = (uint8_t)((msg->flags & 0b0011000) >> 3);
    bool refPosMiss = (msg->flags & 0b0000100) >> 6;
    bool refObsMiss = (msg->flags & 0b0000100) >> 7;

    if (!gnssFixOK || !diffSoln || !relPosValid || carrSoln != 2)
    {
        RCLCPP_INFO(get_logger(), "Dropped at least one GPS update due to flags.\r\nFlags:");
        RCLCPP_INFO(get_logger(), "accuracy: %f2.2", gps_accuracy_m);
        RCLCPP_INFO(get_logger(), "gnssFixOK: %i", gnssFixOK);
        RCLCPP_INFO(get_logger(), "diffSoln: %i", diffSoln);
        RCLCPP_INFO(get_logger(), "relPosValid: %i", relPosValid);
        RCLCPP_INFO(get_logger(), "carrSoln: %i", (int)carrSoln);
        RCLCPP_INFO(get_logger(), "refPosMiss: %i",refPosMiss);
        RCLCPP_INFO(get_logger(), "refObsMiss: %i",refObsMiss);
        return;
    }

    if (!gpsEnabled)
    {
        gpsOdometryValid = false;
        valid_gps_samples = 0;
        return;
    }

    tf2::Vector3 gps_pos(
        getGPSX(gps), getGPSY(gps), getGPSZ(gps));

    handleGPSUpdate(gps_pos, gps_accuracy_m);
}

void ublox_2_odometry::handleGPSUpdate(tf2::Vector3 gps_pos, double gps_accuracy_m)
{

    if (gps_accuracy_m > 0.05)
    {
        RCLCPP_INFO(get_logger(),"dropping gps with accuracy: %f3.4 m",  gps_accuracy_m);
        return;
    }

    double time_since_last_gps = (get_clock()->now() - last_gps_odometry_time).seconds();
    if (time_since_last_gps > 5.0)
    {
        RCLCPP_WARN(get_logger(),"Last GPS was %f3.2 seconds ago", time_since_last_gps);
        gpsOdometryValid = false;
        valid_gps_samples = 0;
        gps_outlier_count = 0;
        last_gps_pos = gps_pos;
        last_gps_acc_m = gps_accuracy_m;
        last_gps_odometry_time = get_clock()->now();
        return;
    }

    double distance_to_last_gps = (last_gps_pos - gps_pos).length();

    if (distance_to_last_gps < 5.0)
    {
        // inlier, we treat it normally

        // calculate current base_link position from orientation and distance parameter

        double base_link_x = gps_pos.x();
        double base_link_y = gps_pos.y();

        // store the gps as last
        last_gps_pos = gps_pos;
        last_gps_acc_m = gps_accuracy_m;
        last_gps_odometry_time = get_clock()->now();

        gps_outlier_count = 0;
        valid_gps_samples++;
        if (!gpsOdometryValid && valid_gps_samples > 10)
        {
            RCLCPP_INFO(get_logger(),"GPS data now valid");
            RCLCPP_INFO(get_logger(),"First GPS data, moving odometry to %f4.2, %f4.2", base_link_x, base_link_y);
            // we don't even have gps yet, set odometry to first estimate
            x = base_link_x;
            y = base_link_y;
            gpsOdometryValid = true;
        }
        else if (gpsOdometryValid)
        {
            // gps was valid before, we apply the filter
            // x = x * (1.0 - config.gps_filter_factor);
            // y = y * (1.0 - config.gps_filter_factor);
            x = base_link_x;
            y = base_link_y;
        }
    }
    else
    {
        RCLCPP_WARN(get_logger(),"GPS outlier found. Distance was: %f3.3", distance_to_last_gps);
        gps_outlier_count++;
        // ~10 sec
        if (gps_outlier_count > 10)
        {
            RCLCPP_WARN(get_logger(),"too many outliers, assuming that the current gps value is valid.");
            last_gps_pos = gps_pos;
            last_gps_acc_m = gps_accuracy_m;
            last_gps_odometry_time = get_clock()->now();

            gpsOdometryValid = false;
            valid_gps_samples = 0;
            gps_outlier_count = 0;
        }
    }

    if (gpsOdometryValid)
    {
        publishOdometry();
    }
}

void ublox_2_odometry::publishOdometry()
{

    rclcpp::Time current_time = get_clock()->now();

    // next, we'll publish the odometry message over ROS
    nav_msgs::msg::Odometry odom;
    odom.header.stamp = current_time;
    odom.header.frame_id = _frame_map;

    // set the position
    odom.pose.pose.position.x = x;
    odom.pose.pose.position.y = y;
    odom.pose.pose.position.z = 0.0;

    odom.pose.pose.orientation = orientation_result;

    // set the velocity
    odom.child_frame_id = _frame_base_link;
    odom.twist.twist.linear.x = vx;
    odom.twist.twist.linear.y = vy;
    odom.twist.twist.angular.z = vr;

    odom.pose.covariance = {
        10000.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 10000.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 10000.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 10000.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0, 0.00001};

    if (gpsOdometryValid && gpsEnabled && get_clock()->now() - last_gps_odometry_time < rclcpp::Duration(5,0))
    {
        odom.pose.covariance[0] = last_gps_acc_m * last_gps_acc_m;
        odom.pose.covariance[7] = last_gps_acc_m * last_gps_acc_m;
    }

    odom.twist.covariance = {
        0.000001, 0.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.000001, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 10000.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 10000.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0, 0.000001};

    // publish the message
   // _odom_pub.publish(odom);
}