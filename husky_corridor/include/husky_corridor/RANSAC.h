#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Point.h>
#include <vector>
#include <string>

struct Line {
    double a, b;                    // Line parameters in 2D space
    std::vector<uint> FittedPoints;
};

class RANSAC {
public:
    RANSAC(const ros::Publisher &pubLine, const ros::Publisher &pubMarker);

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
    Line LinearLeastSquare(const std::vector<uint> &selection);

    // Compute the distance from a point to a line
    double DistanceToLine(
        const geometry_msgs::Point &p,
        const Line &line
    );

    // Publish the line markers for visualization
    void PublishLines(const std::vector<Line> &lines);

    // Publish the points under consideration
    void PublishPoints();

    std::vector<geometry_msgs::Point> PointList;    // In laser frame
    ros::Publisher PubLine, PubMarker;
    std::string LaserFrame;

    const uint ITERATIONS = 200;
    const uint SAMPLE_SIZE = 5;
    const double TOLERANCE = 0.3;
    double CONSENSUS;
    const uint NB_LINES = 4;
    const bool PUBLISH_LINES = true;
    const bool PUBLISH_POINTS = false;
};

// Generate a random sequence between 0 and length
std::vector<uint> GenerateRandomSequence(const uint length);