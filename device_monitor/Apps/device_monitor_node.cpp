

#include <ros/ros.h>
#include "device_monitor/device_monitor.hpp"
int main(int argc, char** argv)
{
    ROS_WARN("*****START*****");
    ros::init(argc, argv, "test_interface");
    ros::NodeHandle nh("~");

    DeviceMonitor dm(nh);
    ros::AsyncSpinner spinner(0);
    spinner.start();

    ros::waitForShutdown();

    return 0;
}
