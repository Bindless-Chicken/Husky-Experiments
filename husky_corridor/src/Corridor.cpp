#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <visualization_msgs/Marker.h>

#include "husky_corridor/RANSAC.h"

int main(int argc, char** argv) {
    ros::init(argc, argv, "husky_corridor");
    ros::NodeHandle nh;

    // Create the publisher to send driving instruction to the husky
    ros::Publisher pubMarker = nh.advertise<visualization_msgs::Marker>("visualization_marker", 0);

    // Create the pillar finder
    RANSAC ransac(pubMarker);

    // Create the subscriber
    ros::Subscriber sub = nh.subscribe("/scan", 10, &RANSAC::Monitor, &ransac);
    ros::spin();

    return 0;
}