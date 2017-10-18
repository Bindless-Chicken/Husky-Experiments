#include "husky_corridor/RANSAC.h"
#include <algorithm>
#include <chrono>
#include <random>
#include <visualization_msgs/Marker.h>

using geometry_msgs::Point;


bool LineFittingCmp(const Line &a, const Line &b)
{
    return a.FittedPoints.size() < b.FittedPoints.size();
}

RANSAC::RANSAC(const ros::Publisher &pubLine) :
    PubLine(pubLine)
{
}

void RANSAC::Monitor(const sensor_msgs::LaserScan::ConstPtr& msg) {
    PointList.clear();

    // Convert the laser range to points
    RangeToCoordinates(
        msg->ranges,
        msg->angle_increment,
        msg->angle_min,
        msg->range_min,
        msg->range_max
    );

    // Do the RANSAC extraction
    std::vector<Line> lines;
    lines = RANSAC2DLine();

    if(PUBLISH_MARKERS) {
        PublishPoints(lines);
    }
}

std::vector<Line> RANSAC::RANSAC2DLine() {
    std::vector<Line> lineList;
    for(uint i = 0; i < ITERATIONS; ++i) {
        std::vector<Point> selection;
        std::vector<Point> test;

        // Generate initial sequence
        std::vector<uint> sequence = GenerateRandomSequence(PointList.size());
        for(uint j = 0; j < PointList.size(); ++j) {
            if(j < SAMPLE_SIZE) {
                selection.push_back(PointList[j]);
            }
            else {
                test.push_back(PointList[j]);
            }
        }

        // Generate initial line
        Line line = LinearLeastSquare(selection);

        // Test fitting
        for(auto tp : test) {
            double distance = DistanceToLine(tp, line);
            // ROS_INFO_STREAM("Distance is " << distance);
            if(distance < TOLERANCE) {
                line.FittedPoints.push_back(tp);
            }
        }

        // ROS_INFO_STREAM("The size  " << line.FittedPoints.size());

        // Refine using fitted points
        Line line2 = LinearLeastSquare(line.FittedPoints);
        line2.FittedPoints = line.FittedPoints;

        lineList.push_back(line2);
    }

    std::sort(lineList.begin(), lineList.end(), LineFittingCmp);

    lineList.resize(NB_LINES);

    return lineList;
}


void RANSAC::RangeToCoordinates(
    const std::vector<float> &ranges,
    const double increment,
    const double start,
    const double min,
    const double max
) {
    for(uint i = 0; i < ranges.size(); ++i) {
        double range = ranges[i];

        // Check that the point is a valid scanner value
        if(range < max && range > min) {
            Point tempPoint;
            tempPoint.x = std::cos(min + (increment * i)) * range;
            tempPoint.y = std::sin(min + (increment * i))  * range;

            PointList.push_back(tempPoint);
        }
    }
}

Line RANSAC::LinearLeastSquare(
    const std::vector<geometry_msgs::Point> &selection
) {
    double sumX = 0;
    double sumY = 0;
    double sumXX = 0;
    double sumYY = 0;
    double sumXY = 0;
    for (int i = 0; i < selection.size(); i++)
    {
        double x = selection[i].x;
        double y = selection[i].y;
        sumX += x;
        sumXX += x * x;
        sumY += y;
        sumYY += y * y;
        sumXY += x * y;
    }
    Line tempLine;

    tempLine.a = (selection.size() * sumXY - sumX * sumY) / (selection.size() * sumXX - sumX * sumX);

    tempLine.b = (sumY * sumXX - sumX * sumXY) / (selection.size() * sumXX - sumX * sumX);

    return tempLine;
}

double RANSAC::DistanceToLine(
    const geometry_msgs::Point &p,
    const Line &line
) {
    double a0 = line.a * p.x;
    double b0 = line.b * p.y;

    double num = std::abs(a0 + b0);

    double denum = std::sqrt(line.a * line.a + line.b * line.b);

    return num / denum;
}

void RANSAC::PublishPoints(const std::vector<Line> &lines) {
    visualization_msgs::Marker line_strip;

    line_strip.header.frame_id = "base_laser";
    line_strip.header.stamp = ros::Time::now();
    line_strip.ns = "lines";

    line_strip.action = visualization_msgs::Marker::ADD;
    line_strip.pose.orientation.w = 1.0;
    line_strip.id = 2;

    line_strip.type = visualization_msgs::Marker::LINE_LIST;

    line_strip.scale.x = 0.1;

    line_strip.color.b = 1.0;
    line_strip.color.a = 1.0;

    for(auto line: lines) {
        Point a;
        a.x = 0;
        a.y = 5 + (line.a * -5 + line.b);

        Point b;
        b.x = 10;
        b.y = 5 + (line.a * 5 + line.b);

        // ROS_INFO_STREAM("Location [" << a.x << ";" << a.y << " - " << b.x << ";" << b.y << "] for fitting: " << lines[0].FittedPoints.size());

        line_strip.points.push_back(a);
        line_strip.points.push_back(b);
    }

    PubLine.publish(line_strip);
}

std::vector<uint> GenerateRandomSequence(const uint length) {
    std::vector<uint> sequence(length);
    for(uint i = 0; i < length; ++i) {
        sequence[i] = i;
    }

    unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
    shuffle(sequence.begin(), sequence.end(), std::default_random_engine(seed));

    return sequence;
}