#include "ublox_2_odometry.h"

ublox_2_odometry::ublox_2_odometry(std::string map_frame, std::string base_frame)
{

    // init ROS stack
    _odom_pub = nh.advertise<nav_msgs::Odometry>("ublox_odometry", 10);
    // _fix_sub = nh.subscribe("ublox/fix", 100, &ublox_2_odometry::fixCallback, this);
    _navlrelposned_sub = nh.subscribe("ublox/navrelposned", 100, &ublox_2_odometry::navrelposnedCallback, this);

    // last_gps_odometry_time(0.0);
    _frame_map = map_frame;
    _frame_base_link = base_frame;

    last_gps_odometry_time = ros::Time(0.0);

    tf2::Quaternion q_mag(0.0, 0.0, 0.0);
    orientation_result = tf2::toMsg(q_mag);
}

double ublox_2_odometry::getGPSY(ublox_msgs::NavRELPOSNED9 &msg)
{
    return ((double)msg.relPosN * 0.01) + ((double)msg.relPosHPN * 0.0001);
}

double ublox_2_odometry::getGPSX(ublox_msgs::NavRELPOSNED9 &msg)
{
    return ((double)msg.relPosE * 0.01) + ((double)msg.relPosHPE * 0.0001);
}

double ublox_2_odometry::getGPSZ(ublox_msgs::NavRELPOSNED9 &msg)
{
    // For now, we assume a plane
    return 0.0;
}

void ublox_2_odometry::fixCallback(const sensor_msgs::NavSatFix::ConstPtr &msg)
{
    double n, e;
    std::string zone;
    RobotLocalization::NavsatConversions::LLtoUTM(msg->latitude, msg->longitude, n, e, zone);

    tf2::Vector3 gps_pos(
        (e - datumE), (n - datumN), 0.0);

    if (msg->position_covariance_type == 0)
    {
        ROS_INFO_STREAM_THROTTLE(1, "Dropped GPS Update to position_covariance_type being UNKNOWN");
        return;
    }

    double acc_m = sqrt(msg->position_covariance[0]);

    handleGPSUpdate(gps_pos, acc_m);
}

void ublox_2_odometry::navrelposnedCallback(const ublox_msgs::NavRELPOSNED9::ConstPtr &msg)
{
    ublox_msgs::NavRELPOSNED9 gps = *msg;
    double gps_accuracy_m = (double)gps.accLength / 10000.0f;

    bool gnssFixOK = (msg->flags & 0b0000001);
    bool diffSoln = (msg->flags & 0b0000010) >> 1;
    bool relPosValid = (msg->flags & 0b0000100) >> 2;
    auto carrSoln = (uint8_t)((msg->flags & 0b0011000) >> 3);
    bool refPosMiss = (msg->flags & 0b0000100) >> 6;
    bool refObsMiss = (msg->flags & 0b0000100) >> 7;

    if (!gnssFixOK || !diffSoln || !relPosValid || carrSoln != 2)
    {
        ROS_INFO_STREAM_THROTTLE(1, "Dropped at least one GPS update due to flags.\r\nFlags:\r\n"
                                        << "accuracy:" << gps_accuracy_m << "\r\n"
                                        << "gnssFixOK:" << gnssFixOK << "\r\n"
                                        << "diffSoln:" << diffSoln << "\r\n"
                                        << "relPosValid:" << relPosValid << "\r\n"
                                        << "carrSoln:" << (int)carrSoln << "\r\n"
                                        << "refPosMiss:" << refPosMiss << "\r\n"
                                        << "refObsMiss:" << refObsMiss << "\r\n");
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
        ROS_INFO_STREAM("dropping gps with accuracy: " << gps_accuracy_m << "m");
        return;
    }

    double time_since_last_gps = (ros::Time::now() - last_gps_odometry_time).toSec();
    if (time_since_last_gps > 5.0)
    {
        ROS_WARN_STREAM("Last GPS was " << time_since_last_gps << " seconds ago.");
        gpsOdometryValid = false;
        valid_gps_samples = 0;
        gps_outlier_count = 0;
        last_gps_pos = gps_pos;
        last_gps_acc_m = gps_accuracy_m;
        last_gps_odometry_time = ros::Time::now();
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
        last_gps_odometry_time = ros::Time::now();

        gps_outlier_count = 0;
        valid_gps_samples++;
        if (!gpsOdometryValid && valid_gps_samples > 10)
        {
            ROS_INFO_STREAM("GPS data now valid");
            ROS_INFO_STREAM("First GPS data, moving odometry to " << base_link_x << ", " << base_link_y);
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
        ROS_WARN_STREAM("GPS outlier found. Distance was: " << distance_to_last_gps);
        gps_outlier_count++;
        // ~10 sec
        if (gps_outlier_count > 10)
        {
            ROS_ERROR_STREAM("too many outliers, assuming that the current gps value is valid.");
            last_gps_pos = gps_pos;
            last_gps_acc_m = gps_accuracy_m;
            last_gps_odometry_time = ros::Time::now();

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

    ros::Time current_time = ros::Time::now();

    // next, we'll publish the odometry message over ROS
    nav_msgs::Odometry odom;
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

    if (gpsOdometryValid && gpsEnabled && ros::Time::now() - last_gps_odometry_time < ros::Duration(5.0))
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
    _odom_pub.publish(odom);
}