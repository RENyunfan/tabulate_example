

#include <ros/ros.h>
#include "device_monitor/device_monitor.hpp"
int main(int argc, char** argv)
{
    ROS_WARN("*****START*****");
    ros::init(argc, argv, "test_node");
    ros::NodeHandle nh("~");

    ros::Rate lp(30);
    ros::Publisher pub = nh.advertise<sensor_msgs::Image>("/test_interface/cam1",1);
    while(ros::ok()){
        sensor_msgs::Image img;
        img.header.stamp = ros::Time::now();
        pub.publish(img);
        lp.sleep();
    }

    return 0;
}
