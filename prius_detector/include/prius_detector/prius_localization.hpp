#pragma once

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>

#include <opencv2/aruco.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

class LocalizationNode {

	public:
    void init(ros::NodeHandle& nh);
    void run();

	private:
    void findGlobalCoordinates();
    void odomCallback(const nav_msgs::Odometry& msg);
    void arrayToMatrixConversion();
		Eigen::Vector3d inMapFrame(Eigen::Vector3d& point);

    ros::Subscriber odom_sub_;
    ros::Publisher global_coord_pub_;

    nav_msgs::Odometry odom_;

    geometry_msgs::PoseStamped pose_;   
		
		Eigen::Matrix3d cameraMatrix, invCameraMatrix;
		Eigen::Matrix3d cameraToQuadMatrix;
		Eigen::Matrix3d quadOrientationMatrix, scaleUpMatrix;
		Eigen::Vector3d translation_, camera_translation_vector_;

    bool debug_;  
    std::vector<float> camera_to_quad_matrix_, camera_translation_,camera_matrix_;
};