#include "calib.h"

/**
 * @brief check how much checkerboard was moved
 *
 * @param x : movement of x-direction
 * @param y : movement of y-direction
 * @param z : movement of z-direction
 * @param rotation  :movement of rotation
 * @return Whether there was Enought Movement or not
 */
int setCalibEnv::thresholdBoardMovement(double &&x, double &&y, double &&z, double &&rotation) {
    for (auto &p : x_movement_DB_) {
        if (std::fabs(p - x) < 5) return CHECK_MOVEMENT::_NOT_ENOUGH_X;
    }
    for (auto &p : y_movement_DB_) {
        if (std::fabs(p - y) < 5) return CHECK_MOVEMENT::_NOT_ENOUGH_Y;
    }
    for (auto &p : z_movement_DB_) {
        if (std::fabs(p - z) < 5) return CHECK_MOVEMENT::_NOT_ENOUGH_Z;
    }
    for (auto &p : rot_movement_DB_) {
        if (std::fabs(p - rotation) < (1 * M_PI / 180.)) return CHECK_MOVEMENT::_NOT_ENOUGH_ROT;
    }
    x_movement_DB_.push_back(x);
    y_movement_DB_.push_back(y);
    z_movement_DB_.push_back(z);
    rot_movement_DB_.push_back(rotation);
    return CHECK_MOVEMENT::_ENOUGH;
}

int setCalibEnv::checkMovement(std::vector<cv::Point2f> &points) {
    double max_x = 0, min_x = 10000, min_y = 10000;
    int max_index, min_index, index = 0;
    for (auto &p : points) {
        if (p.x > max_x) {
            max_x = p.x;
            max_index = index;
        }
        if (p.x < min_x) {
            min_x = p.x;
            min_index = index;
        }
        if (p.y < min_y) {
            min_y = p.y;
        }
        index++;
    }

    return thresholdBoardMovement(std::move(min_x), std::move(min_y), std::move(max_x - min_x),
                                  std::atan2(points[max_index].y - points[min_index].y, points[max_index].x - points[min_index].x));
}

/**
 * @brief callback function of Image raw data
 *
 * @param msg : images data with ros msgs type
 */
void setCalibEnv::calibRawimage(const sensor_msgs::Image::ConstPtr &msg) {
    // cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();   convert cv2 -> ros topic

    std::vector<cv::Point2f> corner_pts, sqr_corner_pts;
    cv::Mat frame, image_resized, gray;

    frame = cv_bridge::toCvShare(msg, "bgr8")->image;
    cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
    // MEMO : resolution check,  std::cout << frame.size().width << "." << frame.size().height << std::endl;
    bool success = cv::findChessboardCorners(gray, cv::Size(checkerboard_colm_num_, checkerboard_rows_num_), corner_pts, CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FAST_CHECK | CV_CALIB_CB_NORMALIZE_IMAGE);
    cv::drawChessboardCorners(frame, cv::Size(checkerboard_colm_num_, checkerboard_rows_num_), corner_pts, success);
    if (success) {
        cv::resize(frame, image_resized, cv::Size(), 0.2, 0.2);
        cv::imshow("Image", image_resized);
        cv::waitKey(10);
        int check_movement = checkMovement(corner_pts);
        if (CHECK_MOVEMENT::_NOT_ENOUGH_X == check_movement) {
            ROS_WARN_STREAM("NEED X-MOVEMENT");
            return;
        }
        if (CHECK_MOVEMENT::_NOT_ENOUGH_Y == check_movement) {
            ROS_WARN_STREAM("NEED Y-MOVEMENT");
            return;
        }
        if (CHECK_MOVEMENT::_NOT_ENOUGH_Z == check_movement) {
            ROS_WARN_STREAM("NEED Y-MOVEMENT");
            return;
        }
        if (CHECK_MOVEMENT::_NOT_ENOUGH_ROT == check_movement) {
            ROS_WARN_STREAM("NEED ROT-MOVEMENT");
            return;
        }

        if (view_cnt_++ < view_num_threshold_) {
            ROS_INFO("DETECT %d", view_cnt_);
            cv::TermCriteria criteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 30, 0.001);
            cv::cornerSubPix(gray, corner_pts, cv::Size(11, 11), cv::Size(-1, -1), criteria);
            objpoints_.push_back(object_);
            imgpoints_.push_back(corner_pts);
            return;
        }
        finish_ = true;

    } else {
        ROS_WARN_STREAM("CAN NOT FIND BOARD CORNER");
        return;
    }

    cv::Mat cameraMatrix, distCoeffs, R, T;
    cv::calibrateCamera(objpoints_, imgpoints_, cv::Size(gray.cols, gray.rows), cameraMatrix, distCoeffs, R, T);
    if (RT_debug_) {
        std::cout << "R : "
                  << R << std::endl;
        std::cout << "T : "
                  << R << std::endl;
    }
    if (intrinsic_debug_) {
        std::cout << "cameraMatrix : \n"
                  << cameraMatrix << std::endl;
        std::cout << "distCoeffs : \n"
                  << distCoeffs << std::endl;
    }

    // COMPLETE #1 : Convert to Json
    if (result_file_type_ == "json") {
        Json::Value intrinsic_json;
        for (int i = 0; i < cameraMatrix.rows; i++) {
            for (int j = 0; j < cameraMatrix.cols; j++) {
                intrinsic_json["intrinsic"][i].append(cameraMatrix.at<double>(i, j));
            }
        }
        for (int i = 0; i < distCoeffs.rows; i++) {
            for (int j = 0; j < distCoeffs.cols; j++) {
                intrinsic_json["distortion"][i].append(distCoeffs.at<double>(i, j));
            }
        }
        Json::Value RT_json;
        for (int i = 0; i < R.rows; i++) {
            for (int j = 0; j < R.cols; j++) {
                RT_json["R"][i].append(R.at<double>(i, j));
            }
        }
        for (int i = 0; i < T.rows; i++) {
            for (int j = 0; j < T.cols; j++) {
                RT_json["T"][i].append(T.at<double>(i, j));
            }
        }

        Json::StyledWriter writer;
        auto str = writer.write(intrinsic_json);
        std::ofstream intrinsic_file(result_file_intrinsic_ + "intrinsic_param.json", std::ofstream::out | std::ofstream::trunc);
        intrinsic_file << str;
        intrinsic_file.close();

        str = writer.write(RT_json);
        std::ofstream RT_file(result_file_RT_ + "RT.json", std::ofstream::out | std::ofstream::trunc);
        RT_file << str;
        RT_file.close();

    } else if (result_file_type_ == "txt") {
        std::ofstream results;
        results.open(result_file_intrinsic_ + "intrinsic_param.txt", std::ofstream::out | std::ofstream::trunc);
        results << cameraMatrix << "\n"
                << distCoeffs;
        results.close();
        results.open(result_file_RT_ + "RT.txt", std::ofstream::out | std::ofstream::trunc);
        results << R << "\n======\n"
                << T;
        results.close();
    }
}

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "calib_node");
    ros::NodeHandle nh("~");
    setCalibEnv calib(nh);

    ros::Rate rate(100);
    while (ros::ok()) {
        // return when finishing calibration
        if (calib.getStatus()) break;
        ros::spinOnce();
    }

    return 0;
}
