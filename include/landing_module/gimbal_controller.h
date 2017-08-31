//
// Created by Eric Fang on 8/7/17.
//

#ifndef LANDING_MODULE_GIMBAL_CONTROLLER_H
#define LANDING_MODULE_GIMBAL_CONTROLLER_H

#include <ros/ros.h>
#include <opencv2/imgproc.hpp>
#include <mavros_msgs/CommandLong.h>

/**
 * controls the gimbal via MAVLINK commands
 */
class gimbal_controller {
private:
    /**
     * internal representation of gimbal state in roll pitch yaw configuration
     * NOTE: Angles are all in local coordinate frame, drone body tilt is internally compensated
     */
    struct rpy {
        double roll_min;
        double roll_max;
        double pitch_min;
        double pitch_max;
        double yaw_min;
        double yaw_max;
        double roll;
        double pitch;
        double yaw;

        rpy(const ros::NodeHandle& nh) {
            double r_min, r_max, p_min, p_max, y_min, y_max;
            nh.param<double>("roll_min", r_min, -45);
            nh.param<double>("roll_max", r_max, 45);
            nh.param<double>("pitch_min", p_min, -90);
            nh.param<double>("pitch_max", p_max, 0);
            nh.param<double>("yaw_min", y_min, -180);
            nh.param<double>("yaw_max", y_max, 180);
            roll_min = r_min;
            roll_max = r_max;
            pitch_min = p_min;
            pitch_max = p_max;
            yaw_min = y_min;
            yaw_max = y_max;
            roll = 0;
            pitch = 0;
            yaw = 0;
        }

        /**
         * add roll angle to gimbal within allowed angles
         * @param input added angle in degrees
         */
        void add_roll(double input) {
            if (roll_min > (roll + input)) roll = roll_min;
            else if ((roll + input) > roll_max) roll = roll_max;
            else roll += input;
        }

        /**
         * add pitch angle to gimbal within allowed angles
         * @param input added angle in degress
         */
        void add_pitch(double input) {
            if (pitch_min > (pitch + input)) pitch = pitch_min;
            else if ((pitch + input) > pitch_max) pitch = pitch_max;
            else pitch += input;
        }

        /**
         * add yaw angle to gimbal within allowed angles
         * @param input added angle in degrees
         */
        void add_yaw(double input) {
            if (yaw_min > (yaw + input)) yaw = (yaw + input) + 2 * yaw_min;
            else if ((yaw + input) > yaw_max) yaw = (yaw + input) + 2 * yaw_max;
            else yaw += input;
        }
    };

    enum MAV_MOUNT_MODE {
        MAV_MOUNT_MODE_RETRACT,
        MAV_MOUNT_MODE_NEUTRAL,
        MAV_MOUNT_MODE_MAVLINK_TARGETING,
        MAV_MOUNT_MODE_RC_TARGETING,
        MAV_MOUNT_MODE_GPS_POINT
    };

    ros::NodeHandle nh_;
    ros::ServiceClient gimbal_command_client;
    ros::Time last_command;
    static const int MAV_CMD_DO_MOUNT_CONTROL = 205;
    static const int MAV_CMD_DO_MOUNT_CONFIGURE = 204;

public:
    rpy gimbal_state;

    gimbal_controller(ros::NodeHandle& nh_private) :
            gimbal_state(nh_private)
    {
        gimbal_command_client = nh_.serviceClient<mavros_msgs::CommandLong>("mavros/cmd/command");
        last_command = ros::Time::now();
    }

    void track_target(cv::Point target_location, cv::Point2i image_size, double h_fov);

    void point_down();
};


#endif //LANDING_MODULE_GIMBAL_CONTROLLER_H
