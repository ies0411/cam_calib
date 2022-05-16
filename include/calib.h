/**
 * @file calib.h
 * @author Eunsoo (eslim@superb-ai.com)
 * @brief mono-cam calibration
 * @version 0.1
 * @date 2022-05-13
 *
 * @copyright Copyright (c) 2022
 *
 */

#ifndef CALIB_CAMERA_HPP
#define CALIB_CAMERA_HPP

#include <cv_bridge/cv_bridge.h>
#include <jsoncpp/json/config.h>
#include <jsoncpp/json/json.h>
#include <jsoncpp/json/writer.h>
#include <opencv2/calib3d/calib3d_c.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <stdio.h>

#include <algorithm>
#include <eigen3/Eigen/Core>
#include <fstream>
#include <iostream>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/opencv_modules.hpp>
#include <vector>
enum CHECK_MOVEMENT {
    _NOT_ENOUGH_X,
    _NOT_ENOUGH_Y,
    _NOT_ENOUGH_Z,
    _NOT_ENOUGH_ROT,
    _ENOUGH,
};

// TODO : roslaunch file and set param
class setCalibEnv {
   private:
    // var
    ros::NodeHandle nh_;
    ros::Subscriber image_sub_;

    int checkerboard_rows_num_, checkerboard_colm_num_;
    std::string image_sub_name_, result_file_intrinsic_, result_file_RT_, result_file_type_;  // MEMO, result file type : 1. json, 2.txt
    double dx_, dy_;                                                                          // MEMO . unit : mm
    int view_cnt_ = 0, view_num_threshold_ = 0;
    bool finish_ = false, RT_debug_, intrinsic_debug_;
    std::vector<double> x_movement_DB_, y_movement_DB_, z_movement_DB_, rot_movement_DB_;

    std::vector<std::vector<cv::Point3f> > objpoints_;
    std::vector<std::vector<cv::Point2f> > imgpoints_;
    std::vector<cv::Point3f> object_;
    // function
    void getParamFunc(ros::NodeHandle &priv_nh);
    void calibRawimage(const sensor_msgs::Image::ConstPtr &msg);
    void setObjectPoint();
    int thresholdBoardMovement(double &&x, double &&y, double &&z, double &&rotation);
    int checkMovement(std::vector<cv::Point2f> &points);

   public:
    setCalibEnv(ros::NodeHandle &priv_nh) {
        getParamFunc(priv_nh);
        setObjectPoint();
        image_sub_ = nh_.subscribe(image_sub_name_, 1, &setCalibEnv::calibRawimage, this);
    }
    bool getStatus() {
        return finish_;
    }

    ~setCalibEnv();
};

setCalibEnv::~setCalibEnv() {
    ROS_INFO_STREAM("terminate");
}

/**
 * @brief push back checkerboard points(world coordinate)
 *
 */
void setCalibEnv::setObjectPoint() {
    for (int i = 0; i < checkerboard_rows_num_; i++) {
        for (int j = 0; j < checkerboard_colm_num_; j++) {
            object_.push_back(cv::Point3f(j * dx_, i * dy_, 0));
        }
    }
}

/**
 * @brief Set the Param from launch file
 *
 * @param priv_nh : private ros node handler
 */
void setCalibEnv::getParamFunc(ros::NodeHandle &priv_nh) {
    priv_nh.param<int>("checker_x_number", checkerboard_colm_num_, 9);
    priv_nh.param<int>("checker_y_number", checkerboard_rows_num_, 6);
    priv_nh.param<double>("dx", dx_, 61.5);
    priv_nh.param<double>("dy", dy_, 61.5);
    priv_nh.param<std::string>("image_sub_name", image_sub_name_, "/pylon_camera_node/image_raw");
    priv_nh.param<int>("view_num", view_num_threshold_, 3);

    priv_nh.param<bool>("RT_debug", RT_debug_, false);
    priv_nh.param<bool>("Intrinsic_debug", intrinsic_debug_, true);
    priv_nh.param<std::string>("result_file_type", result_file_type_, std::string("json"));
    priv_nh.param<std::string>("result_file_intrinsic", result_file_intrinsic_, std::string("/home/catkin_ws/src/mono_cam_calib/result/"));
    priv_nh.param<std::string>("result_file_RT", result_file_RT_, std::string("/home/catkin_ws/src/mono_cam_calib/result/"));
}

#endif
