//
// Created by Eric Fang on 8/7/17.
//

#include <landing_module/search_controller.h>

/**
 * initialize all ROS related callbacks and advertisement of topics
 */
void search_controller::initialize_callbacks() {
    state_sub_ = nh_.subscribe("mavros/state", 10, &search_controller::state_callback, this);
    pos_sub_ = nh_.subscribe("mavros/local_position/pose", 10, &search_controller::pose_callback, this);
    raw_pub_ = nh_.advertise<mavros_msgs::PositionTarget>("mavros/setpoint_raw/local", 10);
    pos_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);
}

/**
 * callback for the current state of the flight controller
 * @param stateMsg
 */
void search_controller::state_callback(const mavros_msgs::StateConstPtr &stateMsg) {
    current_state = *stateMsg;
}

/**
 * callback for the current pose of the UAV
 * @param poseMsg
 */
void search_controller::pose_callback(const geometry_msgs::PoseStampedConstPtr &poseMsg) {
    current_pose = *poseMsg;
}

/**
 * boilerplate code in order to initialize the UAV and switch to OFFBOARD control mode and arm
 * can be offloaded to a higher command node
 */
void search_controller::initialize_uav() {
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

/**
 * main control loop call, lifts the UAV to specified search altitude, then begins search pattern
 * If target is detected search pattern would be interrupted and the UAV would be guided towards the target
 */
void search_controller::controller() {

    mavros_msgs::PositionTarget raw_pos;
    raw_pos.coordinate_frame = raw_pos.FRAME_LOCAL_NED;
    raw_pos.type_mask = raw_pos.IGNORE_AFX | raw_pos.IGNORE_AFY | raw_pos.IGNORE_AFZ | raw_pos.IGNORE_YAW_RATE;
    static unsigned search_index = 0;

    if (!search_mode) {
        raw_pos.type_mask |= raw_pos.IGNORE_VX | raw_pos.IGNORE_VY | raw_pos.IGNORE_VZ | raw_pos.IGNORE_YAW;
        raw_pos.position.x = search_position_x;
        raw_pos.position.y = search_position_y;
        raw_pos.position.z = search_altitude;

        if (std::abs(current_pose.pose.position.z - search_altitude) < 0.5) {
            search_mode = true;
            detector.set_search_mode(true);
        }
    }
    else {
        if (detector.detection_timout() > ros::Duration(10)) {
            raw_pos.type_mask |= raw_pos.IGNORE_VX | raw_pos.IGNORE_VY | raw_pos.IGNORE_VZ;

            raw_pos.position.x = search_position_x + search_pattern[search_index % search_pattern.size()].x;
            raw_pos.position.y = search_position_y + search_pattern[search_index % search_pattern.size()].y;
            raw_pos.position.z = search_altitude;
            raw_pos.yaw = CV_PI/2.0 + std::atan((raw_pos.position.y - current_pose.pose.position.y)/
                                            (raw_pos.position.x - current_pose.pose.position.x));

            if (std::abs(current_pose.pose.position.x - raw_pos.position.x) < 1.0 &&
                std::abs(current_pose.pose.position.y - raw_pos.position.y) < 1.0) {
                search_index++;
            }

            gimbal.point_down();
        }
        else {
            raw_pos.type_mask |= raw_pos.IGNORE_PX | raw_pos.IGNORE_PY | raw_pos.IGNORE_VZ;

            if (detector.detection_timout() < ros::Duration(1)) {
//                gimbal.track_target(detector.get_target_location(), detector.get_image_size(), h_fov);
            }

            //TODO: Implement proper PID control for this
            if (gimbal.gimbal_state.pitch == -90) {
                cv::Point target_location = detector.get_target_location();
                cv::Point2i image_size = detector.get_image_size();

                double error_x = (target_location.x - (image_size.x/2));
                double error_y = (target_location.y - (image_size.y/2));
                double error_corrected_x = error_x * std::cos(-gimbal.gimbal_state.yaw * (CV_PI/180.0)) -
                                           error_y * std::sin(-gimbal.gimbal_state.yaw * (CV_PI/180.0));
                double error_corrected_y = error_x * std::sin(-gimbal.gimbal_state.yaw * (CV_PI/180.0)) +
                                           error_y * std::cos(-gimbal.gimbal_state.yaw * (CV_PI/180.0));

                raw_pos.type_mask |= raw_pos.IGNORE_YAW;
                raw_pos.velocity.x = error_corrected_x * 0.05;
                raw_pos.velocity.y = error_corrected_y * 0.05;
                raw_pos.position.z = search_altitude;
            } else {
                raw_pos.type_mask |= raw_pos.IGNORE_YAW;
//                raw_pos.yaw = gimbal.gimbal_state.yaw * (CV_PI/180.0);
                raw_pos.position.z = search_altitude;
                raw_pos.velocity.y = std::cos(-gimbal.gimbal_state.yaw * (CV_PI/180.0));
                raw_pos.velocity.x = std::sin(-gimbal.gimbal_state.yaw * (CV_PI/180.0));
            }

        }
    }

    raw_pub_.publish(raw_pos);
}

