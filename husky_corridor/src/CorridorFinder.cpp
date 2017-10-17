#include "husky_corridor/CorridorFinder.h"
#include <math.h>


using geometry_msgs::Point;

CorridorFinder::CorridorFinder(const ros::Publisher &pubMarker):
    PubMarker(pubMarker)
{
}

void CorridorFinder::Monitor(const sensor_msgs::LaserScan::ConstPtr& msg) {
    // Take the first laser point
    //      Compute distance of all points to the line between first and second
    //      Add points within tolerance
    //      Keep lines that with the most points


    std::vector<Point> pointList;

    // Fill the point array
    for(uint i = 0; i < msg->ranges.size(); ++i) {
        double range = msg->ranges[i];

        if(range < msg->range_max && range > msg->range_min) {
            Point tempPoint;
            tempPoint.x = std::cos(msg->angle_min + (msg->angle_increment * i)) * range;
            tempPoint.y = std::sin(msg->angle_min + (msg->angle_increment * i))  * range;

            pointList.push_back(tempPoint);
        }
    }

    // Compare points to line
    std::vector<uint> closeList;
    for(uint i = 0; i < pointList.size(); ++i) {
        for(uint j = i+1; j < pointList.size(); ++j) {
            uint close = 0;

            for(uint k = j+1; k < pointList.size(); ++k) {
                double distance = PointLineDistance(pointList[i], pointList[j], pointList[k]);

                // ROS_INFO_STREAM("Point " << k << " is " << distance << " to " << i << ";" << j);

                if(distance < 1) {
                    close++;
                }
            }

            closeList.push_back(close);
        }
    }

    // Sort the close list
    std::sort(closeList.begin(), closeList.end());

    // Draw the first two
    visualization_msgs::Marker line_strip, points;
    line_strip.header.frame_id = points.header.frame_id = msg->header.frame_id;
    line_strip.header.stamp = points.header.stamp = ros::Time::now();
    line_strip.ns = "lines";
    points.ns = "points";
    line_strip.action = points.action = visualization_msgs::Marker::ADD;
    line_strip.pose.orientation.w = points.pose.orientation.w = 1.0;
    line_strip.id = 2;
    points.id =  0;

    line_strip.type = visualization_msgs::Marker::LINE_STRIP;
    points.type = visualization_msgs::Marker::POINTS;

    line_strip.scale.x = 0.1;

    points.scale.x = 0.2;
    points.scale.y = 0.2;

    line_strip.color.b = 1.0;
    line_strip.color.a = 1.0;

    points.color.g = 1.0;
    points.color.a = 1.0;

    points.points = pointList;

    line_strip.points.push_back(pointList[0]);
    line_strip.points.push_back(pointList[150]);

    if(!PubMarker) {
        ROS_WARN("Publisher invalid!");
    }

    PubMarker.publish(line_strip);
    PubMarker.publish(points);
}

double CorridorFinder::PointLineDistance(const Point &p1, const Point &p2, const Point &p3) {
    double dx = p2.x - p1.x;
    double dy = p2.y - p1.y;

    double distance = sqrt(dy * dy + dx * dx);

    double top = abs((dy * p3.x) - (dx * p3.y) + (p2.x * p1.y) - (p2.y * p1.x));

    return top / distance;
}