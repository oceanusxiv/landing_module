//
// Created by Eric Fang on 7/26/17.
//

#include <ros/ros.h>

#include <landing_module/search_controller.h>

/**
 * main loop, runs at 60Hz
 * @param argc
 * @param argv
 * @return
 */
int main(int argc, char** argv) {

    ros::init(argc, argv, "landing_node");
    ros::NodeHandle nh("~");

    ros::Rate rate(60);

    search_controller controller(nh);
    controller.initialize_uav();

//    std::cout << "init complete" << std::endl;

    do {
        controller.controller();
        ros::spinOnce();
        rate.sleep();
    } while(ros::ok());

    return 0;
}