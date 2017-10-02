//
// Created by Eric Fang on 9/14/17.
//

#include <iostream>

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>

#include <image_transport/image_transport.h>

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include <landing_module/target_detector.h>

int main(int argc, char** argv) {

    ros::init(argc, argv, "play_video");
    ros::NodeHandle nh("~");
    image_transport::ImageTransport it(nh);
    image_transport::Publisher pub = it.advertise("camera/image", 10);
    target_detector detector(nh);

    ros::Rate rate(60);

    cv::VideoCapture cap("/Users/eric1221bday/Downloads/GOPR0010_trimmed.mp4");
    cv::Mat frame;
    detector.set_search_mode(true);
    do {
        cap.read(frame);
        pub.publish(cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg());
        ros::spinOnce();
        rate.sleep();
    } while(ros::ok() && cap.isOpened());

    return 0;
}