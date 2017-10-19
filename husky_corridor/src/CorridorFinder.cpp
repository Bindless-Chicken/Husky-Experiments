#include "husky_corridor/CorridorFinder.h"
#include <math.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Twist.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/TransformStamped.h>
#include <visualization_msgs/Marker.h>


using geometry_msgs::Point;

CorridorFinder::CorridorFinder(
    const ros::Publisher &pubHusky,
    const ros::Publisher &pubMarker
):
    PubHusky(pubHusky),
    PubMarker(pubMarker)
{
}

void CorridorFinder::Monitor(const husky_corridor::Lines::ConstPtr& msg) {
    // Select the two first lines and find the intersection point
    if(msg->lines.size() >= 2) {
        geometry_msgs::Point intersection;

        husky_corridor::Line l1 = msg->lines[0];
        husky_corridor::Line l2 = msg->lines[1];

        // Compute the middle point between the two lines in front of the robot
        Point p1;
        p1.x = 2;
        p1.y = l1.a * p1.x + l1.b;

        Point p2;
        p2.x = 2;
        p2.y = l2.a * p2.x + l2.b;

        intersection.x = 1;
        intersection.y = (p1.y + p2.y) / 2;

        // Account for change in coordinates between base_link and base_laser
        intersection.y = -intersection.y;


        geometry_msgs::Twist twistInfo;
        twistInfo.linear.x = 0.5;
        twistInfo.angular.z = atan2(intersection.y, intersection.x);

        PubHusky.publish(twistInfo);


        // Publish the target point marker
        visualization_msgs::Marker points;

        points.header.frame_id = "base_link";
        points.header.stamp = ros::Time::now();
        points.ns = "Target";

        points.action = visualization_msgs::Marker::ADD;
        points.pose.orientation.w = 1.0;
        points.id = 2;

        points.type = visualization_msgs::Marker::POINTS;

        points.scale.x = 0.2;
        points.scale.y = 0.2;

        points.color.r = 1.0;
        points.color.a = 1.0;

        points.points.push_back(intersection);

        PubMarker.publish(points);
    }
}
