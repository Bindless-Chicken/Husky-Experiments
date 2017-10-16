#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Twist.h>
#include <visualization_msgs/Marker.h>

class PillarFinder {
public:
    PillarFinder(const ros::Publisher &pubHusky, const ros::Publisher &pubRViz);

    void Monitor(const sensor_msgs::LaserScan::ConstPtr& msg);

protected:
    uint16_t GetClosestLaser(const sensor_msgs::LaserScan::ConstPtr& msg);
    geometry_msgs::Vector3 ExtractLaserHitLocation(const double range, const double angle);

    ros::Publisher PubHusky;    // Publisher used to send husky movements info
    ros::Publisher PubRViz;     // Publisher used to send markers to RViz

    tf2_ros::Buffer tfBuffer;
    std::string TopicName;
    double queueSize;
};