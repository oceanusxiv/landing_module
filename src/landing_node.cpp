//
// Created by Eric Fang on 7/26/17.
//

#include <ros/ros.h>

#include <landing_module/target_detector.h>

int main(int argc, char** argv) {

    ros::init(argc, argv, "landing_node");
    ros::NodeHandle nh("~");

    target_detector detector(nh);

//    std::cout << "init complete" << std::endl;

    ros::Rate rate(60);

    do {
        ros::spinOnce();
        rate.sleep();
    } while(ros::ok());

    return 0;
}