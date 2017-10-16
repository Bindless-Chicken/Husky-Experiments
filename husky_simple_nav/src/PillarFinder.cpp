#include "husky_simple_nav/PillarFinder.h"
#include <math.h>

PillarFinder::PillarFinder(const ros::Publisher &pubHusky, const ros::Publisher &pubRViz):
    PubHusky(pubHusky),
    PubRViz(pubRViz)
{
}

void PillarFinder::Monitor(const sensor_msgs::LaserScan::ConstPtr& msg) {
    uint16_t rangeID = GetClosestLaser(msg);

    // Get the angle between laser and closest hit
    double hitAngle = msg->angle_min + (rangeID * msg->angle_increment);

    geometry_msgs::Vector3 hitLocation = ExtractLaserHitLocation(msg->ranges[rangeID], hitAngle);

    geometry_msgs::Twist twistInfo;

    // TODO: Something a bit nicer
    twistInfo.linear.x = 2;

    if(hitLocation.y > 0) {
        twistInfo.angular.z = 1;
    }
    else {
        twistInfo.angular.z = -1;
    }
    PubHusky.publish(twistInfo);

    PublishMarker(hitLocation);
}

uint16_t PillarFinder::GetClosestLaser(const sensor_msgs::LaserScan::ConstPtr& msg) {
    float rangeMin = msg->range_max;
    uint iMin = 0;
    for(uint i = 0; i < msg->ranges.size(); ++i){
        if(msg->ranges[i]<rangeMin) {
            rangeMin = msg->ranges[i];
            iMin = i;
        }
    }

    return iMin;
}

geometry_msgs::Vector3 PillarFinder::ExtractLaserHitLocation(const double range, const double angle) {
    geometry_msgs::Vector3 hitLocationScannerSpace;
    hitLocationScannerSpace.x = std::cos(angle) * range;
    hitLocationScannerSpace.y = std::sin(angle) * range;
    tf2_ros::TransformListener tfListener(tfBuffer);

    // Transform scanner space to base space
    geometry_msgs::Vector3 returnVal;
    geometry_msgs::TransformStamped ts;
    try {
        ts = tfBuffer.lookupTransform("base_link", "base_laser", ros::Time(0));

        returnVal.x = hitLocationScannerSpace.x + ts.transform.translation.x;
        returnVal.y = -hitLocationScannerSpace.y + ts.transform.translation.y;
    } catch(tf2::TransformException &ex) {
        ROS_WARN("%s", ex.what());
        ros::Duration(1.0).sleep();
    }

    return returnVal;
}

void PillarFinder::PublishMarker(const geometry_msgs::Vector3& location) {
    double cylinderRadius = 0.2;

    geometry_msgs::TransformStamped ts;
    tf2_ros::TransformListener tfListener(tfBuffer);
    try {
        geometry_msgs::PointStamped locationOdomSpace;
        geometry_msgs::PointStamped point_tf;
        point_tf.header.frame_id = "base_link";
        point_tf.header.stamp = ros::Time();
        point_tf.point.x = location.x;
        point_tf.point.y = location.y;
        point_tf.point.z = location.z;


        // // ts = tfBuffer.lookupTransform("odom", "base_link", ros::Time());
        // tfBuffer.transform(point_tf, locationOdomSpace, "odom");

        // // Add the current robot location
        // geometry_msgs::TransformStamped robotLocation = tfBuffer.lookupTransform("odom", "base_link", ros::Time(0));

        // locationOdomSpace.point.x -= robotLocation.transform.translation.x;
        // locationOdomSpace.point.y -= robotLocation.transform.translation.y;
        // locationOdomSpace.point.z -= robotLocation.transform.translation.z;


        // ROS_INFO("Object is at [%f, %f, %f]",
        //     robotLocation.transform.translation.x,
        //     robotLocation.transform.translation.y,
        //     robotLocation.transform.translation.z
        // );

        // locationOdomSpace.x = locationOdomSpace.x + ts.transform.translation.x;
        // locationOdomSpace.y = locationOdomSpace.y + ts.transform.translation.y;
        // locationOdomSpace.z = locationOdomSpace.z + ts.transform.translation.z;

        // Create the info for the marker
        visualization_msgs::Marker pillarMarker;
        pillarMarker.header.frame_id = "base_link";
        pillarMarker.header.stamp = ros::Time();
        pillarMarker.ns = "husky_highlevel_controller";
        pillarMarker.id = 0;
        pillarMarker.type = visualization_msgs::Marker::SPHERE;
        pillarMarker.action = visualization_msgs::Marker::ADD;
        pillarMarker.pose.position = point_tf.point;
        pillarMarker.scale.x = 1;
        pillarMarker.scale.y = 1;
        pillarMarker.scale.z = 1;
        pillarMarker.color.a = 1.0;
        pillarMarker.color.r = 1.0;

        PubRViz.publish(pillarMarker);
    } catch(tf2::TransformException &ex) {
        ROS_WARN("%s", ex.what());
        ros::Duration(1.0).sleep();
    }
}