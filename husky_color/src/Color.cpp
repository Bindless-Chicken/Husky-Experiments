#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>


void Monitor(const sensor_msgs::ImageConstPtr &msg) {
    cv_bridge::CvImagePtr cvPtr;

    try {
        cvPtr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch(cv_bridge::Exception &e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
    }

    cv::Mat inverted = ~cvPtr->image;
    cv::Mat hsvImage;
    cv::cvtColor(inverted, hsvImage, cv::COLOR_BGR2HSV);

    cv::Mat1b mask;
    cv::inRange(hsvImage, cv::Scalar(90 - 20, 70, 0), cv::Scalar(90 + 20, 255, 255), mask);

    cv::imshow("Image Original", cvPtr->image);
    cv::imshow("Image", mask);
    cv::waitKey(3);
}


int main(int argc, char **argv) {
    ros::init(argc, argv, "husky_color");
    ros::NodeHandle nh;

    image_transport::ImageTransport it(nh);
    image_transport::Subscriber subImage;

    subImage = it.subscribe("/camera/rgb/image_raw", 10, Monitor);

    cv::namedWindow("Image Original");
    cv::namedWindow("Image");

    ros::spin();
}