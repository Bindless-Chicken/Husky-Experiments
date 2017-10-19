#include "husky_corridor/RANSAC.h"
#include <algorithm>
#include <chrono>
#include <random>
#include <visualization_msgs/Marker.h>
#include "husky_corridor/Lines.h"

using geometry_msgs::Point;


bool LineFittingCmp(const Line &a, const Line &b)
{
    return a.FittedPoints.size() < b.FittedPoints.size();
}

RANSAC::RANSAC(const ros::Publisher &pubLine, const ros::Publisher &pubMarker) :
    PubLine(pubLine),
    PubMarker(pubMarker)
{
}

void RANSAC::Monitor(const sensor_msgs::LaserScan::ConstPtr& msg) {
    PointList.clear();
    LaserFrame = msg->header.frame_id;

    // Convert the laser range to points
    RangeToCoordinates(
        msg->ranges,
        msg->angle_increment,
        msg->angle_min,
        msg->range_min,
        msg->range_max
    );

    ReduceDensity();

    // Do the RANSAC extraction
    std::vector<Line> lines;
    lines = RANSAC2DLine();

    husky_corridor::Lines tempLines;

    for(auto line: lines) {
        husky_corridor::Line tempLine;
        tempLine.a = line.a;
        tempLine.b = line.b;

        tempLine.header.frame_id = LaserFrame;
        tempLine.header.stamp = ros::Time::now();

        tempLines.lines.push_back(tempLine);
    }

    PubLine.publish(tempLines);

    if(PUBLISH_LINES) {
        PublishLines(lines);
    }

    if(PUBLISH_POINTS) {
        PublishPoints();
    }
}

std::vector<Line> RANSAC::RANSAC2DLine() {
    std::vector<Line> lineList;
    CONSENSUS = PointList.size()/4;

    for(uint i = 0; i < ITERATIONS; ++i) {
        if(PointList.size() > CONSENSUS + 10) {
            std::vector<uint> selection;
            std::vector<uint> test;

            // Generate initial sequence
            std::vector<uint> sequence = GenerateRandomSequence(PointList.size());
            for(uint j = 0; j < PointList.size(); ++j) {
                if(j < SAMPLE_SIZE) {
                    selection.push_back(sequence[j]);
                }
                else {
                    test.push_back(sequence[j]);
                }
            }

            // Generate initial line
            Line line = LinearLeastSquare(selection);

            // Test fitting
            for(auto tp : test) {
                double distance = DistanceToLine(PointList[tp], line);
                if(distance < TOLERANCE) {
                    line.FittedPoints.push_back(tp);
                }
            }


            // Refine using fitted points
            if(line.FittedPoints.size() > CONSENSUS) {
                Line line2 = LinearLeastSquare(line.FittedPoints);
                line2.FittedPoints = line.FittedPoints;

                // Remove that points that have already been matched
                std::sort(line2.FittedPoints.begin(), line2.FittedPoints.end());

                for(int j = line2.FittedPoints.size() - 1; j > 0; --j)
                {
                    PointList.erase(PointList.begin() + line2.FittedPoints[j]);
                }

                lineList.push_back(line2);
            }
        }
    }

    std::sort(lineList.begin(), lineList.end(), LineFittingCmp);

    if(lineList.size() > NB_LINES) {
        lineList.resize(NB_LINES);
    }

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
            tempPoint.x = std::cos(start + (increment * i)) * range;
            tempPoint.y = std::sin(start + (increment * i))  * range;

            PointList.push_back(tempPoint);
        }
    }
}

#define SQR(x) ((x)*(x))

void RANSAC::ReduceDensity() {
    uint idOrig = 0;
    for(uint i = 1; i < PointList.size(); ++i) {
        Point orig = PointList[idOrig];
        Point stud = PointList[i];

        double distance = std::sqrt(
            SQR(stud.x - orig.x) +
            SQR(stud.y - orig.y)
        );

        if(distance < DENSITY)  {
            PointList.erase(PointList.begin() + i);
            --i;
        }
        else {
            idOrig = i;
        }
    }
}

Line RANSAC::LinearLeastSquare(
    const std::vector<uint> &selection
) {
    double sumX = 0;
    double sumY = 0;
    double sumXX = 0;
    double sumYY = 0;
    double sumXY = 0;
    for (int i = 0; i < selection.size(); i++)
    {
        double x = PointList[selection[i]].x;
        double y = PointList[selection[i]].y;
        sumX += x;
        sumXX += x * x;
        sumY += y;
        sumYY += y * y;
        sumXY += x * y;
    }
    Line tempLine;

    double denum = (selection.size() * sumXX) - (sumX * sumX);

    if(denum == 0) {
        ROS_ERROR("Cannot solve the matrix!");
    }

    tempLine.a = (selection.size() * sumXY - sumX * sumY) / denum;
    tempLine.b = (sumY * sumXX - sumX * sumXY) / denum;

    return tempLine;
}

double RANSAC::DistanceToLine(
    const geometry_msgs::Point &p,
    const Line &line
) {
    double ao = -1.0 / line.a;
    double bo = p.y - ao * p.x;
    double px = (line.b - bo) / (ao - line.a);
    double py = ((ao * (line.b - bo)) / (ao -line.a)) + bo;
    return std::sqrt(std::pow(p.x - px, 2) + std::pow(p.y - py, 2));
}

void RANSAC::PublishLines(const std::vector<Line> &lines) {
    visualization_msgs::Marker line_strip;

    line_strip.header.frame_id = LaserFrame;
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
        a.x = -5;
        a.y = line.a * -5 + line.b;

        Point b;
        b.x = 5;
        b.y = line.a * 5 + line.b;

        line_strip.points.push_back(a);
        line_strip.points.push_back(b);
    }

    PubMarker.publish(line_strip);
}

void RANSAC::PublishPoints() {
    visualization_msgs::Marker points;

    points.header.frame_id = LaserFrame;
    points.header.stamp = ros::Time::now();
    points.ns = "points";

    points.action = visualization_msgs::Marker::ADD;
    points.pose.orientation.w = 1.0;
    points.id = 2;

    points.type = visualization_msgs::Marker::POINTS;

    points.scale.x = 0.2;
    points.scale.y = 0.2;

    points.color.g = 1.0;
    points.color.a = 1.0;

    points.points = PointList;

    PubMarker.publish(points);
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