#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

#include "husky_simple_nav/PillarFinder.h"
#include "husky_simple_nav/EmergencyStop.h"

#define PKG_NAMESPACE "/husky_simple_nav/"

using husky_simple_nav::EmergencyStop;

int main(int argc, char** argv) {
    ros::init(argc, argv, "husky_simple_nav");
    ros::NodeHandle nh;

    // Retrieve some config variables
    std::string  topicName = "/scan";
    if( !nh.getParam(PKG_NAMESPACE "topic_name", topicName)) {
        ROS_WARN("Topic name config was not found, using default [/scan]");
    }
    double queueSize = 10;
    if( !nh.getParam(PKG_NAMESPACE "queue_size", queueSize)) {
        ROS_WARN("Queue size config was not found, using default [10]");
    }

    // Create the publisher to send driving instruction to the husky
    ros::Publisher pubHusky = nh.advertise<geometry_msgs::Twist>("/husky_velocity_controller/cmd_vel", 1);

    // Create the publisher to send marker info to RViz
    ros::Publisher pubEmer = nh.advertise<EmergencyStop>("emergency_stop", 0);

    // Create the pillar finder
    PillarFinder finder(pubHusky, pubEmer);

    // Create the subscriber
    ros::Subscriber sub = nh.subscribe(topicName, queueSize, &PillarFinder::Monitor, &finder);
    ros::spin();

    return 0;
}