//
// Created by Eric Fang on 7/26/17.
//

#include <landing_module/target_detector.h>

/**
 * initializes all ROS related callbacks and advertisements
 */
void target_detector::initialize_callbacks() {
    image_sub_ = it_.subscribe("/play_video/camera/image", 1, &target_detector::images_callback, this);
    image_pub_ = it_.advertise("/out", 1);
}

/**
 * main detection function for 2x2 checkerboard detection
 * current algorithm: corners would be extracted from the input image, and each corner would be subject to the below
 * filtering and identification.
 * 1. Extract a 5x5 square ring centered on the corner detection
 * 2. Eliminate if the ring does not contain 4 transitions, defined as a difference between two pixels larger than the
 * mean pixel value of the ring.
 * 3. Separate the ring into 4 blocks divided by the transition point
 * 4. Eliminate if alternating blocks are not within a threshold value, and neighboring blocks are not above a threshold
 * value
 * 5. Output corner location as center of checkerboard if all tests are passed
 * @param input camera image as a grayscale
 * @param display image in color used for visualization of detection
 * @return whether there has been a target detection or not
 */
bool target_detector::detect_target(const cv::Mat &input, const cv::Mat& display) {

    std::vector<cv::Point2f> corners;
    int maxCorners = 50;
    double qualityLevel = 0.01;
    double minDistance = 5;
    int blockSize = 3;
    bool useHarrisDetector = false;
    double k = 0.04;

    cv::goodFeaturesToTrack(input, corners, maxCorners, qualityLevel, minDistance, cv::Mat(),
                            blockSize, useHarrisDetector, k);

    for (auto& corner : corners) {
        // edge case rejection
        if (corner.x < 2 || corner.y < 2 || (corner.x > (input.cols - 2)) || (corner.y > (input.rows - 2))) return false;
        cv::circle(display, corner, 3, cv::Scalar(255, 0, 0), -1, 8, 0);
        std::vector<int> transitions;
        std::vector<int> pixels;
        // extracting ring of pixel centered on corner
        for (auto& loc : ring) {
            pixels.push_back(int(input.at<uchar>(corner + loc)));
        }

        double sum = std::accumulate(pixels.begin(), pixels.end(), 0.0);
        double mean = sum / pixels.size();

        for (int i = 1; i < pixels.size(); i++) {
            if (pixels[i] > mean && pixels[i - 1] <= mean) {
                transitions.push_back(i);

            }
            else if (pixels[i] < mean && pixels[i - 1] >= mean) {
                transitions.push_back(i);
            }
        }

        if (transitions.size() == 4) {
            int num_first = (transitions[0] + int(pixels.size() - 1) - transitions[3]);
            int num_second = (transitions[1] - transitions[0]);
            int num_third = (transitions[2] - transitions[1]);
            int num_fourth = (transitions[3] - transitions[2]);

            if (num_first < 2 || num_second < 2 || num_third < 2 || num_fourth < 2) return false;

            int sum_first = std::accumulate(pixels.begin(), pixels.begin() + transitions[0], 0);
            sum_first += std::accumulate(pixels.begin() + transitions[3], pixels.end() - 1, 0);
            int sum_second = std::accumulate(pixels.begin() + transitions[0], pixels.begin() + transitions[1], 0);
            int sum_third = std::accumulate(pixels.begin() + transitions[1], pixels.begin() + transitions[2], 0);
            int sum_fourth = std::accumulate(pixels.begin() + transitions[2], pixels.begin() + transitions[3], 0);

            int mean_first = sum_first / num_first;
            int mean_second = sum_second / num_second;
            int mean_third = sum_third / num_third;
            int mean_fourth = sum_fourth / num_fourth;

            // alternating blocks must be roughly the same color
            if ((std::abs(mean_first - mean_third) > close_threshold) || (std::abs(mean_second - mean_fourth) > close_threshold)) {
                return false;
            }
            // neighboring blocks must be roughly the opposite color
            if ((std::abs(mean_first - mean_second) < far_threshold) || (std::abs(mean_third - mean_fourth) < far_threshold)) {
                return false;
            }

            target_location.x = corner.x;
            target_location.y = corner.y;

            cv::circle(display, corner, 3, cv::Scalar(255, 0, 0), -1, 8, 0);
            std::ostringstream text;
            text << transitions.size();
            cv::putText(display, text.str(), corner, fontFace, 0.5, cv::Scalar(255, 0, 0));
            return true;
        }
    }

    return false;
}

/**
 * callback for the camera image, the detecion is also done here and the visualization image is published
 * @param imageMsg
 */
void target_detector::images_callback(const sensor_msgs::ImageConstPtr &imageMsg) {
    cv_bridge::CvImagePtr src_gray_ptr;
    cv_bridge::CvImagePtr src_ptr;

    if (search_mode) {
        try {
            src_gray_ptr = cv_bridge::toCvCopy(imageMsg, sensor_msgs::image_encodings::MONO8);
            src_ptr = cv_bridge::toCvCopy(imageMsg, sensor_msgs::image_encodings::BGR8);
        } catch (cv_bridge::Exception &e) {
            throw std::runtime_error(
                    std::string("cv_bridge exception: ") + std::string(e.what()));

        }

        create_ring(2);
        std::cout << ring.size() << std::endl;
        target_found = detect_target(src_gray_ptr->image, src_ptr->image);

        if (target_found) {
            if (log) log_detection(src_ptr->image, src_gray_ptr->image);
            last_detection = ros::Time::now();
        }

        image_pub_.publish(src_ptr->toImageMsg());
    }
}

/**
 * test function used for analysis by saving the image patch centered on detected targets as well as the whole image
 * @param image
 * @param gray_image
 */
void target_detector::log_detection(cv::Mat &image, cv::Mat &gray_image) {
    std::stringstream image_file_name;
    std::stringstream patch_file_name;
    std::stringstream time;

    time << ros::Time::now();
    image_file_name << "/Users/eric1221bday/sandbox/" << time.str() << ".jpg";
    patch_file_name << "/Users/eric1221bday/sandbox/" << time.str() << "_patch.jpg";

    gray_image.at<uchar>(target_location) = 255;
    cv::Mat success_patch = gray_image(cv::Rect(std::max(int(target_location.x - 9), 0),
                                                std::max(int(target_location.y - 9), 0), 20, 20));
    cv::imwrite(image_file_name.str(), image);
    cv::imwrite(patch_file_name.str(), success_patch);
}

void target_detector::create_ring(int radius) {
    ring.clear();

    for (int i = radius; i > -radius; i--) {
        ring.emplace_back(i, radius);
    }
    for (int i = radius; i > -radius; i--) {
        ring.emplace_back(-radius, i);
    }
    for (int i = -radius; i < radius; i++) {
        ring.emplace_back(i, -radius);
    }
    for (int i = -radius; i < radius; i++) {
        ring.emplace_back(radius, i);
    }
}