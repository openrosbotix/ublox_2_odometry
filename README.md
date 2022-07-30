# ublox_2_odometry
Translation of ublox/navrelposned9 message to nav_msgs/odometry

This ROS node can be used to translate a ublox/navrelposned9 message into a common nav_msgs/odometry. 
NavRELPOSNED9 provides relative coordinates between a rover and the base station. This gets translated into coordinates of nav_msgs/odometry. 
This way, you can simple use the output in robot_localization as absolute reference and there is no need to use navsat_transform node anymore.

This node only publishes data, if a valid gps fix is available
