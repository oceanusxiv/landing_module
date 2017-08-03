//
// Created by Eric Fang on 7/26/17.
//

#ifndef LANDING_MODULE_TARGET_DETECTOR_H
#define LANDING_MODULE_TARGET_DETECTOR_H

#include <numeric>

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/CommandLong.h>
#include <message_filters/subscriber.h>
#include <cv_bridge/cv_bridge.h>

#include <image_transport/subscriber_filter.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

class target_detector {
private:
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
        }

        void add_roll(double input) {
            if (roll_min > (roll + input)) roll = roll_min;
            else if ((roll + input) > roll_max) roll = roll_max;
            else roll += input;
        }

        void add_pitch(double input) {
            if (pitch_min > (pitch + input)) pitch = pitch_min;
            else if ((pitch + input) > pitch_max) pitch = pitch_max;
            else pitch += input;
        }

        void add_yaw(double input) {
            if (yaw_min > (yaw + input)) yaw = yaw_min;
            else if ((yaw + input) > yaw_max) yaw = yaw_max;
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

    ros::NodeHandle nh_private_;
    ros::NodeHandle nh_;
    ros::Time last_command;
    mavros_msgs::State current_state;
    geometry_msgs::PoseStamped current_pose;
    image_transport::ImageTransport it_;
//    message_filters::Subscriber<geometry_msgs::PoseStamped> pose_sub_;
//    image_transport::SubscriberFilter image_sub_;
    image_transport::Subscriber image_sub_;
    image_transport::Publisher image_pub_;
    ros::Subscriber state_sub_;
    ros::Subscriber pos_sub_;
    ros::Publisher pos_pub_;
    ros::Publisher vel_pub_;
    ros::ServiceClient gimbal_command_client;
    ros::ServiceClient arming_client;
    ros::ServiceClient set_mode_client;
    typedef message_filters::sync_policies::ApproximateTime<
            geometry_msgs::PoseStamped, sensor_msgs::Image> tracker_policy;
//    message_filters::Synchronizer<tracker_policy> synchronizer_;
    std::vector<cv::Point2f> ring {cv::Point2f(2, 0),
                                   cv::Point2f(2, 1),
                                   cv::Point2f(2, 2),
                                   cv::Point2f(1, 2),
                                   cv::Point2f(0, 2),
                                   cv::Point2f(-1, 2),
                                   cv::Point2f(-2, 2),
                                   cv::Point2f(-2, 1),
                                   cv::Point2f(-2, 0),
                                   cv::Point2f(-2, -1),
                                   cv::Point2f(-2, -2),
                                   cv::Point2f(-1, -2),
                                   cv::Point2f(0, -2),
                                   cv::Point2f(1, -2),
                                   cv::Point2f(2, -2),
                                   cv::Point2f(2, -1),
                                   cv::Point2f(2, 0)};
    rpy gimbal_state;
    static const int MAV_CMD_DO_MOUNT_CONTROL = 205;
    static const int MAV_CMD_DO_MOUNT_CONFIGURE = 204;
    const int fontFace = CV_FONT_NORMAL;
    double h_fov;
    double search_altitude, search_position_x, search_position_y, search_yaw;

    void topics_callback(/*const geometry_msgs::PoseStampedConstPtr& poseMsg,*/
                         const sensor_msgs::ImageConstPtr& imageMsg);

    void state_callback(const mavros_msgs::StateConstPtr& stateMsg);

    void pose_callback(const geometry_msgs::PoseStampedConstPtr& poseMsg);

    bool detect_target(const cv::Mat &input, const cv::Mat& display, cv::Point2f& result);

    void track_target(cv::Point target_location, const cv::Mat &image);

public:
    target_detector(const ros::NodeHandle& nh_private) :
            nh_private_(nh_private),
            it_(nh_private),
            gimbal_state(nh_private)
//            pose_sub_(nh_private, "pose", 1),
//            image_sub_(it_, "image", 1)
//            synchronizer_(tracker_policy(10), pose_sub_, image_sub_)
    {
        initialize_callbacks();
        gimbal_command_client = nh_.serviceClient<mavros_msgs::CommandLong>("mavros/cmd/command");
        arming_client = nh_.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
        set_mode_client = nh_.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
        nh_private_.param<double>("h_fov", h_fov, 90);
        nh_private_.param<double>("search_altitude", search_altitude, 30);
        nh_private_.param<double>("search_position_x", search_position_x, 0);
        nh_private_.param<double>("search_position_y", search_position_y, 0);
        nh_private_.param<double>("search_yaw", search_yaw, 0);

        last_command = ros::Time::now();
    }
    ~target_detector() = default;

    void search_controller();

    void initialize_callbacks();

    void initialize_uav();

    static geometry_msgs::Quaternion toQuaternion(double pitch, double roll, double yaw)
    {
        geometry_msgs::Quaternion q;
        double t0 = std::cos(yaw * 0.5);
        double t1 = std::sin(yaw * 0.5);
        double t2 = std::cos(roll * 0.5);
        double t3 = std::sin(roll * 0.5);
        double t4 = std::cos(pitch * 0.5);
        double t5 = std::sin(pitch * 0.5);

        q.w = t0 * t2 * t4 + t1 * t3 * t5;
        q.x = t0 * t3 * t4 - t1 * t2 * t5;
        q.y = t0 * t2 * t5 + t1 * t3 * t4;
        q.z = t1 * t2 * t4 - t0 * t3 * t5;
        return q;
    }
};


#endif //LANDING_MODULE_TARGET_DETECTOR_H
