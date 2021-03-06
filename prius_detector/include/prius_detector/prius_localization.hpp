#pragma once

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <tf/tf.h>

#include <cmath>
#include <iostream>
#include <opencv2/aruco.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <utility>
#include <vector>

class LocalizationNode {
  public:
    void init(ros::NodeHandle& nh);
    void run();
    void findGlobalCoordinates();
    void odomCallback(const nav_msgs::Odometry& msg);
    void arrayToMatrixConversion();
    Eigen::Vector3d inMapFrame(Eigen::Vector3d& point);
    // float getOrientationOfPrius(std::vector<std::pair<float, float>>& corners, std::pair<float, float> centre);
    void getOrientationOfPrius(Eigen::Vector3d& midPointOfFrontWheels, Eigen::Vector3d& centre);
    void getVelocityOfPrius(Eigen::Vector3d& currentPositionOfPrius);
    void publishSetPoint();
    void publishVelocity();
    
    ros::Subscriber odom_sub_;
    ros::Publisher global_coord_pub_;
    ros::Publisher odom_pub_;
    ros::Publisher vel_pub_;
    ros::Publisher setpt_pub_;
    ros::Publisher drone_vel_pub_;

    nav_msgs::Odometry odom_;
    nav_msgs::Odometry odom_drone_;
    geometry_msgs::Twist prius_vel_;

    geometry_msgs::PoseStamped pose_;

    Eigen::Matrix3d cameraMatrix, invCameraMatrix;
    Eigen::Matrix3d cameraToQuadMatrix;
    Eigen::Matrix3d quadOrientationMatrix, scaleUpMatrix;
    Eigen::Vector3d translation_, camera_translation_vector_;

    Eigen::Vector3d previous_position_, current_position_;
    double previous_time_ , current_time_;
    double current_yaw_, previous_yaw_, drone_yaw_;

    bool debug_;
    std::vector<float> camera_to_quad_matrix_, camera_translation_, camera_matrix_;
};