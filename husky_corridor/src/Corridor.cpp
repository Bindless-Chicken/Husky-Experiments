#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <visualization_msgs/Marker.h>

#include <geometry_msgs/Twist.h>

#include "husky_corridor/RANSAC.h"
#include "husky_corridor/CorridorFinder.h"
#include "husky_corridor/Lines.h"
int main(int argc, char** argv) {
    ros::init(argc, argv, "husky_corridor");
    ros::NodeHandle nh;

    // Create the publisher to send driving instruction to the husky
    ros::Publisher pubMarker = nh.advertise<visualization_msgs::Marker>("visualization_marker", 0);

    // Create the publisher to send the lines
    ros::Publisher pubLines = nh.advertise<husky_corridor::Lines>("identified_lines", 0);

    // Create the publisher to control the husky
    ros::Publisher pubHusky = nh.advertise<geometry_msgs::Twist>("/husky_velocity_controller/cmd_vel", 1);

    // Create RANSAC listener
    RANSAC ransac(pubLines, pubMarker);

    // Create the behaviour node
    CorridorFinder finder(pubHusky);

    // Create the subscribers
    ros::Subscriber sub = nh.subscribe("/scan", 10, &RANSAC::Monitor, &ransac);
    ros::Subscriber sub2 = nh.subscribe("/identified_lines", 10, &CorridorFinder::Monitor, &finder);

    ros::spin();

    return 0;
}