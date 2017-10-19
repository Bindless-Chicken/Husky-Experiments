#include <ros/ros.h>
#include "husky_corridor/Lines.h"
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

class CorridorFinder {
public:
    CorridorFinder(
        const ros::Publisher &pubHusky,
        const ros::Publisher &pubMarker
    );

    void Monitor(const husky_corridor::Lines::ConstPtr& msg);

protected:
    ros::Publisher PubHusky;   // Publisher used to send line marker
    ros::Publisher PubMarker;
    tf2_ros::Buffer tfBuffer;
};