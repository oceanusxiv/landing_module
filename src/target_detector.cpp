//
// Created by Eric Fang on 7/26/17.
//

#include <landing_module/target_detector.h>

void target_detector::initialize_callbacks() {
//    synchronizer_.registerCallback(boost::bind(&target_detector::topics_callback, this, _1, _2));
    image_sub_ = it_.subscribe("image", 1, &target_detector::topics_callback, this);
    image_pub_ = it_.advertise("/out", 1);
    state_sub_ = nh_.subscribe("mavros/state", 10, &target_detector::state_callback, this);
    pos_sub_ = nh_.subscribe("mavros/local_position/pose", 10, &target_detector::pose_callback, this);
    pos_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);
    vel_pub_ = nh_.advertise<geometry_msgs::TwistStamped>("mavros/setpoint_velocity/cmd_vel", 10);
}

void target_detector::state_callback(const mavros_msgs::StateConstPtr &stateMsg) {
    current_state = *stateMsg;
}

void target_detector::pose_callback(const geometry_msgs::PoseStampedConstPtr &poseMsg) {
    current_pose = *poseMsg;
}

void target_detector::initialize_uav() {
    ros::Rate rate(20.0);

    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }

    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = 0;

    //send a few setpoints before starting
    for(int i = 100; ros::ok() && i > 0; --i){
        pos_pub_.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::SetMode set_mode_msg;
    set_mode_msg.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    while(ros::ok() && !current_state.armed) {

        if( current_state.mode != "OFFBOARD" &&
            (ros::Time::now() - last_command > ros::Duration(5.0))){
            if( set_mode_client.call(set_mode_msg) &&
                set_mode_msg.response.success) {
                ROS_INFO("Offboard enabled");
            }
            last_command = ros::Time::now();
        }
        else {
            if( !current_state.armed &&
                (ros::Time::now() - last_command > ros::Duration(5.0))){
                if( arming_client.call(arm_cmd) &&
                    arm_cmd.response.success) {
                    ROS_INFO("Vehicle armed");
                }
                last_command = ros::Time::now();
            }
        }

        pos_pub_.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }
}

void target_detector::search_controller() {

    geometry_msgs::TwistStamped twist;

    twist.twist.linear.x = 0;
    twist.twist.linear.y = 0;
    twist.twist.linear.z = search_altitude - current_pose.pose.position.z;

    if (ros::Time::now() - last_detection > ros::Duration(2)) {

        mavros_msgs::CommandLong gimbal_command;
        static double yaw_dir = 45;

        if (gimbal_state.yaw == gimbal_state.yaw_max) yaw_dir = -45;
        else if (gimbal_state.yaw == gimbal_state.yaw_min) yaw_dir = 45;

        gimbal_state.add_yaw(yaw_dir);
        gimbal_state.pitch = -60;

        gimbal_command.request.command = MAV_CMD_DO_MOUNT_CONTROL;
        gimbal_command.request.param1 = float(gimbal_state.pitch);
        gimbal_command.request.param3 = float(gimbal_state.yaw);
        gimbal_command.request.param7 = MAV_MOUNT_MODE_MAVLINK_TARGETING;

        if (ros::Time::now() - last_command > ros::Duration(2)) {

            if (gimbal_command_client.call(gimbal_command) && !int(gimbal_command.response.success)) {
                std::cout << "command failed!" << std::endl;
            }

            last_command = ros::Time::now();
        }
    }

    vel_pub_.publish(twist);

//    geometry_msgs::PoseStamped pose;
//
//    pose.pose.position.x = search_position_x;
//    pose.pose.position.y = search_position_y;
//    pose.pose.position.z = search_altitude;
//    pose.pose.orientation = toQuaternion(0, 0, search_yaw);
//
//    pos_pub_.publish(pose);

}

bool target_detector::detect_target(const cv::Mat &input, const cv::Mat& display, cv::Point2f& result) {

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
        if (corner.x < 2 || corner.y < 2) return false;

        std::vector<int> transitions;
        std::vector<double> pixels;
        for (auto& loc : ring) {
            pixels.push_back(double(input.at<uchar>(corner + loc)));
        }

        double sum = std::accumulate(pixels.begin(), pixels.end(), 0.0);
        double mean = sum / pixels.size();
        int previous = 0;

        for (int i = 1; i < pixels.size(); i++) {
            if (pixels[i] > mean && pixels[i - 1] <= mean) {
                transitions.push_back(i);
//                if (previous == 1) break;
//                if (previous == 0) previous = 1;
//                if (previous == -1) previous = 1;

            }
            else if (pixels[i] < mean && pixels[i - 1] >= mean) {
                transitions.push_back(i);
//                if (previous == -1) break;
//                if (previous == 0) previous = -1;
//                if (previous == 1) previous = -1;

            }
        }

        cv::circle(display, corner, 3, cv::Scalar(255, 0, 0), -1, 8, 0);
        std::ostringstream text;
        text << transitions.size();
        cv::putText(display, text.str(), corner, fontFace, 0.5, cv::Scalar(255, 0, 0));

        if (transitions.size() == 4) {
            result.x = corner.x;
            result.y = corner.y;
            return true;
        }
    }

    return false;
}

void target_detector::track_target(cv::Point target_location, const cv::Mat &image) {

    double aspect_ratio = double(image.cols)/image.rows;
    double v_fov = h_fov/aspect_ratio;
    double yaw_correction = (target_location.x - (image.cols/2))*(h_fov/image.cols);
    double pitch_correction = -(target_location.y - (image.rows/2))*(v_fov/image.rows);
    gimbal_state.add_yaw(yaw_correction);
    gimbal_state.add_pitch(pitch_correction);

//    std::cout << gimbal_state.yaw << ", " << gimbal_state.pitch << std::endl;

    mavros_msgs::CommandLong gimbal_command;

    gimbal_command.request.broadcast = 0;
    gimbal_command.request.confirmation = 0;
    gimbal_command.request.command = MAV_CMD_DO_MOUNT_CONTROL;
    gimbal_command.request.param1 = float(gimbal_state.pitch);
    gimbal_command.request.param2 = 0;
    gimbal_command.request.param3 = float(gimbal_state.yaw);
    gimbal_command.request.param4 = 0;
    gimbal_command.request.param5 = 0;
    gimbal_command.request.param6 = 0;
    gimbal_command.request.param7 = MAV_MOUNT_MODE_MAVLINK_TARGETING;

    if (ros::Time::now() - last_command > ros::Duration(1)) {

        if (gimbal_command_client.call(gimbal_command) && !int(gimbal_command.response.success)) {
            std::cout << "command failed!" << std::endl;
        }

        last_command = ros::Time::now();
    }
}

void target_detector::topics_callback(/*const geometry_msgs::PoseStampedConstPtr& poseMsg,*/
                                      const sensor_msgs::ImageConstPtr& imageMsg) {
    cv_bridge::CvImageConstPtr src_gray_ptr;
    cv_bridge::CvImagePtr src_ptr;

    if (current_state.armed) {
        try {
            src_gray_ptr = cv_bridge::toCvCopy(imageMsg, sensor_msgs::image_encodings::MONO8);
            src_ptr = cv_bridge::toCvCopy(imageMsg, sensor_msgs::image_encodings::BGR8);
        } catch (cv_bridge::Exception &e) {
            throw std::runtime_error(
                    std::string("cv_bridge exception: ") + std::string(e.what()));

        }

        cv::Point2f target_location;

        target_found = detect_target(src_gray_ptr->image, src_ptr->image, target_location);

        if (target_found) {
            cv::Mat success_patch = src_gray_ptr->image(cv::Rect(std::max(int(target_location.x - 2), 0), std::max(int(target_location.y - 2), 0), 4, 4));

            cv::Mat dst_x, dst_y, dst, abs_dst_x, abs_dst_y, patch_enlarged;

            cv::resize(success_patch, patch_enlarged, cv::Size(32, 32), 0, 0, cv::INTER_NEAREST);
            cv::GaussianBlur(patch_enlarged, patch_enlarged, cv::Size(3, 3), 0, 0);
            cv::Scharr(patch_enlarged, dst_x, CV_32F, 1, 0);
            cv::Scharr(patch_enlarged, dst_y, CV_32F, 0, 1);
            cv::convertScaleAbs(dst_x, abs_dst_x);
            cv::convertScaleAbs(dst_y, abs_dst_y);
            cv::addWeighted(abs_dst_x, 0.5, abs_dst_y, 0.5, 0, dst);
    //    cv::Mat src_display = src_gray_ptr->image(cv::Rect(600, 350, 100, 100));
            cv::imshow("Window", dst);
            cv::imshow("Window2", patch_enlarged);
            cv::waitKey(3);
        }

        // Output modified video stream
        image_pub_.publish(src_ptr->toImageMsg());

        if (target_found) {
            last_detection = ros::Time::now();
            track_target(target_location, src_ptr->image);
        }
    }


}