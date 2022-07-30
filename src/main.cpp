#include "ublox_2_odometry.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ublox2Odometry");

    ublox_2_odometry ublox2odom("map", "base_link");

    ros::AsyncSpinner spinner(0);
    spinner.start();

    ros::Rate rate(10.0);
    int rate_counter = 0;

    while (ros::ok())
    {
        rate.sleep();
    }

    return 0;
}
