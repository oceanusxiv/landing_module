//
// Created by Eric Fang on 7/26/17.
//

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>

#include <landing_module/target_detector.h>

int main(int argc, char** argv) {

    ros::init(argc, argv, "landing_node");
    ros::NodeHandle nh("~");

    ros::Rate rate(60);

    target_detector detector(nh);
    detector.initialize_uav();

//    std::cout << "init complete" << std::endl;

    do {
        detector.search_controller();
        ros::spinOnce();
        rate.sleep();
    } while(ros::ok());

    return 0;
}