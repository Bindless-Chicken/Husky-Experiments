#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>

class CorridorFinder {
public:
    CorridorFinder(const ros::Publisher &pubMarker);

    void Monitor(const sensor_msgs::LaserScan::ConstPtr& msg);

protected:
    double PointLineDistance(const geometry_msgs::Point &p1, const geometry_msgs::Point &p2, const geometry_msgs::Point &p3);

    ros::Publisher PubMarker;   // Publisher used to send line marker
};