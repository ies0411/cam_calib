#include "stereo.h"

void setCalibEnv::calibStereoRawimage(const mono_cam_calib::stereo::ConstPtr& msg) {
    std::vector<cv::Point2f> corner_pts_left, corner_pts_right, sqr_corner_pts_left, sqr_corner_pts_right;
    cv::Mat frame_left, frame_right, gray_left, gray_right;
    // sensor_msgs::Image left, right;
    // left = msg->data[_LEFT];
    // right = msg->data[_RIGHT];
    // frame = cv_bridge::toCvShare(msg, "bgr8")->image;

    frame_left = cv_bridge::toCvShare(msg->data[_LEFT], msg, "bgr8")->image;
    frame_right = cv_bridge::toCvShare(msg->data[_RIGHT], msg, "bgr8")->image;

    cvtColor(frame_left, gray_left, cv::COLOR_BGR2GRAY);
    cvtColor(frame_left, gray_right, cv::COLOR_BGR2GRAY);

    bool success_left = cv::findChessboardCorners(gray_left, cv::Size(checkerboard_colm_num_, checkerboard_rows_num_), corner_pts_left, CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FAST_CHECK | CV_CALIB_CB_NORMALIZE_IMAGE);
    bool success_right = cv::findChessboardCorners(gray_right, cv::Size(checkerboard_colm_num_, checkerboard_rows_num_), corner_pts_right, CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FAST_CHECK | CV_CALIB_CB_NORMALIZE_IMAGE);

    if (!success_left || !success_right) {
        ROS_WARN_STREAM("CAN NOT FIND BOARD CORNER");
        return;
    }

    cv::TermCriteria criteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 30, 0.001);
    cv::cornerSubPix(gray_left, corner_pts_left, cv::Size(11, 11), cv::Size(-1, -1), criteria);
    cv::drawChessboardCorners(gray_left, cv::Size(checkerboard_colm_num_, checkerboard_rows_num_), corner_pts_left, success_left);
    cv::imshow("left_image", frame_left);
    cv::cornerSubPix(gray_right, corner_pts_right, cv::Size(11, 11), cv::Size(-1, -1), criteria);
    cv::drawChessboardCorners(gray_right, cv::Size(checkerboard_colm_num_, checkerboard_rows_num_), corner_pts_right, success_right);
    cv::imshow("right_image", frame_right);
    if (view_cnt_++ < view_num_threshold_) {
        ROS_INFO("DETECT %d", view_cnt_);
        objpoints_right_.push_back(object_);
        objpoints_left_.push_back(object_);
        imgpoints_right_.push_back(corner_pts_right);
        imgpoints_left_.push_back(corner_pts_left);
        return;
    }
    finish_ = true;

    cv::Mat cameraMatrix_left, cameraMatrix_right, distCoeffs_left, distCoeffs_right, R_left, R_right, T_left, T_right;
    cv::calibrateCamera(objpoints_left_, imgpoints_left_, cv::Size(gray_left.rows, gray_left.cols), cameraMatrix_left, distCoeffs_left, R_left, T_left);
    cv::calibrateCamera(objpoints_right_, imgpoints_right_, cv::Size(gray_right.cols, gray_right.rows), cameraMatrix_right, distCoeffs_right, R_right, T_right);

    cv::Mat R, T, E, F;
    cv::stereoCalibrate(objpoints_left_, imgpoints_left_, imgpoints_right_,
                        cameraMatrix_left, distCoeffs_left, cameraMatrix_right, distCoeffs_right, cv::Size(gray_left.rows, gray_left.cols),
                        R, T, E, F, cv::CALIB_FIX_INTRINSIC, cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30, 1e-6));

    cv::Mat R1, R2, P1, P2, Q;
    cv::Rect validRoi[2];

    cv::stereoRectify(cameraMatrix_left, distCoeffs_left, cameraMatrix_right, distCoeffs_right, cv::Size(gray_left.rows, gray_left.cols),
                      R, T, R1, R2, P1, P2, Q, cv::CALIB_ZERO_DISPARITY, 1, cv::Size(gray_left.rows, gray_left.cols), &validRoi[0], &validRoi[1]);

    cv::Mat imgU1, imgU2, lmapx, lmapy, rmapx, rmapy;
    cv::initUndistortRectifyMap(cameraMatrix_left, distCoeffs_left, R1, P1, cv::Size(gray_left.rows, gray_left.cols), CV_32F, lmapx, lmapy);
    cv::initUndistortRectifyMap(cameraMatrix_right, distCoeffs_right, R2, P2, cv::Size(gray_left.rows, gray_left.cols), CV_32F, rmapx, rmapy);
    cv::remap(frame_left, imgU1, lmapx, lmapy, cv::INTER_LINEAR);
    cv::remap(frame_right, imgU2, rmapx, rmapy, cv::INTER_LINEAR);

    cv::imwrite(save_path, imgU1);
    cv::imwrite(save_path, imgU2);
}

int main(int argc, char* argv[]) {
    ros::init(argc, argv, "stereo_node");
    ros::NodeHandle nh("~");
    setCalibEnv calib(nh);
    ros::Rate rate(100);
    int idx = 0, size;
    std::vector<cv::String> images_left, images_right;
    std::array<std::string, 2> path = std::move(calib.getStereoImagefilepath());
    // std::string right_path = std::move(calib.getStereoImagefilepath());
    if (calib.getImagefileStatus()) {
        cv::glob(path[STEREO::_LEFT], images_left);
        cv::glob(path[STEREO::_RIGHT], images_right);
        if (images_left.size() != images_right.size()) {
            ROS_WARN("left images number is different from right images number");
            exit(-1);
        }
        int size = images_left.size();
        std::sort(images_left.begin(), images_left.end());
        std::sort(images_right.begin(), images_right.end());
        std::cout << "number of images : " << size << std::endl;
    }

    while (ros::ok()) {
        // return when finishing calibration
        if (calib.getImagefileStatus()) {
            cv::Mat frame_right = cv::imread(images_left[idx++]);
            cv::Mat frame_left = cv::imread(images_right[idx++]);
            calib.convertCVtoROS(frame_right);
            calib.convertCVtoROS(frame_left);
            calib.publishStereoImg();
            if (idx == size) {
                calib.setStatus(true);
            }
        }
        if (calib.getStatus()) break;
        ros::spinOnce();
        rate.sleep();
    }
    // TODO : 컴파일 할때도 stereo옵션 flag주기
    return 0;
}
