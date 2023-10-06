#include "ublox_2_odometry.hpp"

int main(int argc, char **argv)
{

  rclcpp::init(argc, argv);

    auto ublox2odom = std::make_shared<ublox_2_odometry>("map", "base_link");

    rclcpp::Rate rate(10.0);
    while (rclcpp::ok())
    {
        rclcpp::spin_some(ublox2odom);
        rate.sleep();
    }

    return 0;
}
