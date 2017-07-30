//
// Created by Eric Fang on 7/26/17.
//

#include <landing_module/target_detector.h>

void target_detector::initializeCallbacks() {
    synchronizer_.registerCallback(boost::bind(&target_detector::topics_callback, this, _1, _2));
    image_pub_ = it_.advertise("/out", 1);
}

bool target_detector::detect_target(const cv::Mat &input, cv::Point2f& result) {

    std::vector<cv::Point2f> corners;
    int maxCorners = 10;
    double qualityLevel = 0.01;
    double minDistance = 5;
    int blockSize = 3;
    bool useHarrisDetector = false;
    double k = 0.04;
    double threshold = 100;

    cv::goodFeaturesToTrack(input, corners, maxCorners, qualityLevel, minDistance, cv::Mat(),
                            blockSize, useHarrisDetector, k);

    for (auto& corner : corners) {
        int transitions = 0;
        for (int i = 1; i < ring.size(); i++) {
            if (std::abs(input.at<uchar>(corner + ring[i]) - input.at<uchar>(corner + ring[i - 1])) > threshold) transitions++;
        }

        if (transitions == 4) {
            result.x = corner.x;
            result.y = corner.y;
            return true;
        }
    }

    return false;
}

void target_detector::topics_callback(const geometry_msgs::PoseStampedConstPtr& poseMsg,
                                      const sensor_msgs::ImageConstPtr& imageMsg) {

    cv_bridge::CvImageConstPtr src_gray_ptr;
    cv_bridge::CvImagePtr src_ptr;

    try {
        src_gray_ptr = cv_bridge::toCvCopy(imageMsg, sensor_msgs::image_encodings::MONO8);
        src_ptr = cv_bridge::toCvCopy(imageMsg, sensor_msgs::image_encodings::BGR8);
    } catch (cv_bridge::Exception &e) {
        throw std::runtime_error(
                std::string("cv_bridge exception: ") + std::string(e.what()));

    }

    cv::Mat dst = src_gray_ptr->image.clone();
    cv::Point2f target_location;

    bool success = detect_target(src_gray_ptr->image, target_location);

//    if (success) cv::circle(src_ptr->image, target_location, 5, cv::Scalar(255, 0, 0), -1, 8, 0);
//    cv::Mat src_display;
//    cv::resize(src_ptr->image, src_display, cv::Size(960, 540));
//    cv::imshow("Window", src_display);
//    cv::waitKey(3);
//
//    // Output modified video stream
//    image_pub_.publish(src_ptr->toImageMsg());

    if (success) {
        double yaw_correction = (target_location.x - (src_ptr->image.cols/2))*(h_fov/src_ptr->image.cols);
        double pitch_correction = -(target_location.y - (src_ptr->image.rows/2))*(v_fov/src_ptr->image.rows);
        gimbal_state.yaw += yaw_correction;
        gimbal_state.pitch += pitch_correction;

        std::cout << yaw_correction << ", " << pitch_correction << std::endl;

        mavros_msgs::CommandLong gimbal_command;
        gimbal_command.request.command = MAV_CMD_DO_MOUNT_CONTROL;
        gimbal_command.request.param1 = 0;
        gimbal_command.request.param2 = 0;
        gimbal_command.request.param3 = 0;
        gimbal_command.request.param7 = MAV_MOUNT_MODE_MAVLINK_TARGETING;
//        gimbal_command_client.call(gimbal_command);
    }
}