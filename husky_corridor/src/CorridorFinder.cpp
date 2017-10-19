#include "husky_corridor/CorridorFinder.h"
#include <math.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Twist.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/TransformStamped.h>


using geometry_msgs::Point;

CorridorFinder::CorridorFinder(const ros::Publisher &pubHusky):
    PubHusky(pubHusky)
{
}

void CorridorFinder::Monitor(const husky_corridor::Lines::ConstPtr& msg) {
    // Select the two first lines and find the intersection point
    if(msg->lines.size() >= 2) {
        geometry_msgs::Point intersection;

        husky_corridor::Line l1 = msg->lines[0];
        husky_corridor::Line l2 = msg->lines[1];

        intersection.x = (l2.b - l1.b) / (l1.a - l2.a);
        intersection.y = l1.a * intersection.x + l1.b;

        // tf2_ros::TransformListener tfListener(tfBuffer);
        // geometry_msgs::TransformStamped ts;
        // ts = tfBuffer.lookupTransform("base_link", "base_laser", ros::Time(0));

        // intersection.x += ts.transform.translation.x;
        intersection.y = -intersection.y /*+ ts.transform.translation.y*/;

        geometry_msgs::Twist twistInfo;
        twistInfo.linear.x = 0.5;
        twistInfo.angular.z = (atan2(intersection.y, intersection.x)) / 2.0;

        PubHusky.publish(twistInfo);
    }
}
