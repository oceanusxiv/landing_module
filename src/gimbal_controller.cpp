//
// Created by Eric Fang on 8/7/17.
//

#include <landing_module/gimbal_controller.h>

/**
 * commands the gimbal to track the specified target, keeping it in the center of the gimbaled image
 * @param target_location target location in image pixel location
 * @param image_size image size in pixels
 * @param h_fov horizontal fov of the input image
 */
void gimbal_controller::track_target(cv::Point target_location, cv::Point2i image_size, double h_fov) {
    double aspect_ratio = double(image_size.x)/image_size.y;
    double v_fov = h_fov/aspect_ratio;
    double yaw_correction = (target_location.x - (image_size.x/2))*(h_fov/image_size.x)*0.5;
    double pitch_correction = -(target_location.y - (image_size.y/2))*(v_fov/image_size.y)*0.5;
    gimbal_state.add_yaw(yaw_correction);
    gimbal_state.add_pitch(pitch_correction);

//    std::cout << gimbal_state.yaw << ", " << gimbal_state.pitch << std::endl;

    mavros_msgs::CommandLong gimbal_command;

    gimbal_command.request.command = MAV_CMD_DO_MOUNT_CONTROL;
    gimbal_command.request.param1 = float(gimbal_state.pitch);
    gimbal_command.request.param2 = 0;
    gimbal_command.request.param3 = float(gimbal_state.yaw);
    gimbal_command.request.param7 = MAV_MOUNT_MODE_MAVLINK_TARGETING;

    if (ros::Time::now() - last_command > ros::Duration(0.5)) {

        if (gimbal_command_client.call(gimbal_command) && !int(gimbal_command.response.success)) {
            std::cout << "command failed!" << std::endl;
        }

        last_command = ros::Time::now();
    }
}

/**
 * commands the gimbal to point downwards in local coordinate frame
 */
void gimbal_controller::point_down() {
    mavros_msgs::CommandLong gimbal_command;

    if (ros::Time::now() - last_command > ros::Duration(0.5)) {

        gimbal_state.pitch = -90;

        gimbal_command.request.command = MAV_CMD_DO_MOUNT_CONTROL;
        gimbal_command.request.param1 = float(gimbal_state.pitch);
        gimbal_command.request.param3 = float(gimbal_state.yaw);
        gimbal_command.request.param7 = MAV_MOUNT_MODE_MAVLINK_TARGETING;

        if (gimbal_command_client.call(gimbal_command) && !int(gimbal_command.response.success)) {
            std::cout << "command failed!" << std::endl;
        }

        last_command = ros::Time::now();
    }
}