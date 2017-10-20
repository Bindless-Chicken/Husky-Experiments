#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>


void Monitor(const sensor_msgs::ImageConstPtr &msg) {
    cv_bridge::CvImagePtr cvPtr;

    try {
        cvPtr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch(cv_bridge::Exception &e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
    }

    cv::imshow("Image", cvPtr->image);
    cv::waitKey(3);
}


int main(int argc, char **argv) {
    ros::init(argc, argv, "husky_color");
    ros::NodeHandle nh;

    image_transport::ImageTransport it(nh);
    image_transport::Subscriber subImage;

    subImage = it.subscribe("/camera/rgb/image_raw", 10, Monitor);

    cv::namedWindow("Image");

    ros::spin();
}