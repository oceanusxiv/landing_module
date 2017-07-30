//
// Created by Eric Fang on 7/26/17.
//

#ifndef LANDING_MODULE_TARGET_DETECTOR_H
#define LANDING_MODULE_TARGET_DETECTOR_H

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/PoseStamped.h>
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
        double roll;
        double pitch;
        double yaw;
    };

    enum MAV_MOUNT_MODE {
        MAV_MOUNT_MODE_RETRACT,
        MAV_MOUNT_MODE_NEUTRAL,
        MAV_MOUNT_MODE_MAVLINK_TARGETING,
        MAV_MOUNT_MODE_RC_TARGETING,
        MAV_MOUNT_MODE_GPS_POINT
    };

    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    message_filters::Subscriber<geometry_msgs::PoseStamped> pose_sub_;
    image_transport::SubscriberFilter image_sub_;
    image_transport::Publisher image_pub_;
    ros::ServiceClient gimbal_command_client;
    typedef message_filters::sync_policies::ApproximateTime<
            geometry_msgs::PoseStamped, sensor_msgs::Image> tracker_policy;
    message_filters::Synchronizer<tracker_policy> synchronizer_;
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
    double aspect_ratio = 1.77;
    double h_fov = 90;
    double v_fov = h_fov/aspect_ratio;

    void initializeCallbacks();
    void topics_callback(const geometry_msgs::PoseStampedConstPtr& poseMsg,
                         const sensor_msgs::ImageConstPtr& imageMsg);
    bool detect_target(const cv::Mat& input, cv::Point2f& result);

public:
    target_detector(ros::NodeHandle nh) :
            nh_(nh),
            it_(nh),
            pose_sub_(nh, "pose", 1),
            image_sub_(it_, "image", 1),
            synchronizer_(tracker_policy(10), pose_sub_, image_sub_)
    {
        initializeCallbacks();
        gimbal_command_client = nh_.serviceClient<mavros_msgs::CommandLong>("cmd/command");
    }
    ~target_detector() = default;
};


#endif //LANDING_MODULE_TARGET_DETECTOR_H
