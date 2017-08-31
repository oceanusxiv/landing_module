//
// Created by Eric Fang on 7/26/17.
//

#ifndef LANDING_MODULE_TARGET_DETECTOR_H
#define LANDING_MODULE_TARGET_DETECTOR_H

#include <numeric>
#include <iostream>
#include <fstream>

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>

#include <image_transport/image_transport.h>

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

/**
 * detects a 2x2 checkerboard target, optimized for high distance low resolution detection
 */
class target_detector {
private:
    ros::NodeHandle nh_private_;
    ros::NodeHandle nh_;
    ros::Time last_detection;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    image_transport::Publisher image_pub_;
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
    const int fontFace = CV_FONT_NORMAL;
    double h_fov, aspect_ratio;
    bool target_found, search_mode, log;
    const int close_threshold = 25;
    const int far_threshold = 100;
    cv::Point2i image_size  = cv::Point2i(1280, 720);
    cv::Point target_location;

    void images_callback(const sensor_msgs::ImageConstPtr &imageMsg);

    bool detect_target(const cv::Mat &input, const cv::Mat& display);

    void log_detection(cv::Mat& image, cv::Mat& gray_image);

public:
    target_detector(const ros::NodeHandle& nh_private) :
            nh_private_(nh_private),
            it_(nh_private),
            log(false)
    {
        initialize_callbacks();
        nh_private_.param<double>("h_fov", h_fov, 90);
    }

    ~target_detector() = default;

    void initialize_callbacks();

    /**
     *
     * @return the time since last detection of target
     */
    ros::Duration detection_timout() {
        return ros::Duration(ros::Time::now() - last_detection);
    }

    /**
     *
     * @return get the last known target location in image pixel coordinates
     */
    cv::Point get_target_location() {
        return target_location;
    }

    /**
     *
     * @return the size of the incoming image in pixels
     */
    cv::Point2i get_image_size() {
        return image_size;
    }

    /**
     *
     * @param mode whether the craft is in search mode or not
     */
    void set_search_mode(bool mode) {
        search_mode = mode;
    }
};


#endif //LANDING_MODULE_TARGET_DETECTOR_H
