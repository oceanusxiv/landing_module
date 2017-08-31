//
// Created by Eric Fang on 8/7/17.
//

#ifndef LANDING_MODULE_SEARCH_CONTROLLER_H
#define LANDING_MODULE_SEARCH_CONTROLLER_H

#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>

#include <landing_module/target_detector.h>
#include <landing_module/gimbal_controller.h>

/**
 * controller in charge of searching for the target, includes both search pattern and homing to target
 */
class search_controller {
private:
    ros::NodeHandle nh_private_;
    ros::NodeHandle nh_;
    ros::Subscriber state_sub_, pos_sub_;
    ros::Publisher raw_pub_, pos_pub_;
    ros::Time last_command;
    mavros_msgs::State current_state;
    geometry_msgs::PoseStamped current_pose;
    ros::ServiceClient arming_client;
    ros::ServiceClient set_mode_client;
    std::vector<cv::Point2f> search_pattern{cv::Point2f(-50, -50),
                                            cv::Point2f(-50, 50),
                                            cv::Point2f(50, 50),
                                            cv::Point2f(50, -50)};
    target_detector detector;
    gimbal_controller gimbal;

    double search_altitude, search_position_x, search_position_y, h_fov;
    bool search_mode;

    void initialize_callbacks();

public:
    search_controller(ros::NodeHandle& nh_private) :
            nh_private_(nh_private),
            detector(nh_private),
            gimbal(nh_private),
            search_mode(false),
            last_command(ros::Time::now())
    {
        initialize_callbacks();
        arming_client = nh_.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
        set_mode_client = nh_.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
        nh_private_.param<double>("search_altitude", search_altitude, 35);
        nh_private_.param<double>("search_position_x", search_position_x, 0);
        nh_private_.param<double>("search_position_y", search_position_y, 0);
        nh_private_.param<double>("h_fov", h_fov, 90);
    }

    void controller();

    void initialize_uav();

    void state_callback(const mavros_msgs::StateConstPtr& stateMsg);

    void pose_callback(const geometry_msgs::PoseStampedConstPtr& poseMsg);
};


#endif //LANDING_MODULE_SEARCH_CONTROLLER_H
