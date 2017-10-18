#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Point.h>
#include <vector>

struct Line {
    double a, b;                    // Line parameters in 2D space
    std::vector<geometry_msgs::Point> FittedPoints;
};

class RANSAC {
public:
    RANSAC(const ros::Publisher &pubLine);

    // Callback answering to a new message from the laser
    void Monitor(const sensor_msgs::LaserScan::ConstPtr& msg);

protected:

    std::vector<Line> RANSAC2DLine();

    // Fill the member PointList with the given ranges
    void RangeToCoordinates(
        const std::vector<float> &ranges,
        const double increment,
        const double start,
        const double min,
        const double max
    );

    // Find the line fitting to a maximum of inputs
    Line LinearLeastSquare(const std::vector<geometry_msgs::Point> &selection);

    // Compute the distance from a point to a line
    double DistanceToLine(
        const geometry_msgs::Point &p,
        const Line &line
    );

    // Publish the line markers for visualization
    void PublishPoints(const std::vector<Line> &lines);

    std::vector<geometry_msgs::Point> PointList;    // In laser frame
    ros::Publisher PubLine;

    const uint ITERATIONS = 30;
    const uint SAMPLE_SIZE = 20;
    const uint TOLERANCE = 2;
    const uint NB_LINES = 2;
    const bool PUBLISH_MARKERS = true;
};

// Generate a random sequence between 0 and length
std::vector<uint> GenerateRandomSequence(const uint length);