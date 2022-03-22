#include <prius_detector/prius_localization.hpp>

void LocalizationNode::init(ros::NodeHandle& nh) {
    odom_sub_ = nh.subscribe("/mavros/local_position/odom", 1, &LocalizationNode::odomCallback, this);
    odom_pub_ = nh.advertise<nav_msgs::Odometry>("/prius_odom", 50);
    vel_pub_ = nh.advertise<geometry_msgs::Twist>("/prius_vel", 50);
    setpt_pub_ = nh.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_position/local",50);
    drone_vel_pub_ = nh.advertise<geometry_msgs::TwistStamped>("/mavros/setpoint_velocity/cmd_vel",50);

    // set parameters
    nh.getParam("/camera_matrix", camera_matrix_);
    nh.getParam("/cam_to_quad_rot", camera_to_quad_matrix_);
    nh.getParam("/t_cam", camera_translation_);

    arrayToMatrixConversion();
    debug_ = true;
    previous_position_ = Eigen::Vector3d(0,0,1);
    previous_time_ = 0;
    previous_yaw_ = 0;

}

void LocalizationNode::arrayToMatrixConversion() {
    for (int i = 0; i < 3; i++) {
        camera_translation_vector_(i) = camera_translation_[i];
        for (int j = 0; j < 3; j++) {
            cameraToQuadMatrix(i, j) = camera_to_quad_matrix_[3 * i + j];
            cameraMatrix(i, j) = camera_matrix_[3 * i + j];
        }
    }

    invCameraMatrix = cameraMatrix.inverse();
}

void LocalizationNode::odomCallback(const nav_msgs::Odometry& msg) {
    odom_drone_ = msg;
    tf::Quaternion q(msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w);
    Eigen::Quaterniond quat = Eigen::Quaterniond(q.w(), q.x(), q.y(), q.z());
    quadOrientationMatrix = quat.normalized().toRotationMatrix();
    translation_ = Eigen::Vector3d(msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z);

    double roll, pitch, yaw;
    tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
    drone_yaw_ = yaw;
}

Eigen::Vector3d LocalizationNode::inMapFrame(Eigen::Vector3d& point) {
    Eigen::Vector3d quad_frame_coordinates = cameraToQuadMatrix * point + camera_translation_vector_;
    std::cout << "quad frame coordinates" << quad_frame_coordinates(0) << " " << quad_frame_coordinates(1) << " " << quad_frame_coordinates(2) << std::endl; 
    Eigen::Vector3d map_frame_coordinates = quadOrientationMatrix * quad_frame_coordinates + translation_ ;

    return map_frame_coordinates;
}

void LocalizationNode::getOrientationOfPrius(Eigen::Vector3d& midPointOfFrontWheels, Eigen::Vector3d& centre) {

    current_yaw_ = std::atan2((midPointOfFrontWheels(1) - centre(1)) , (midPointOfFrontWheels(0) - centre(0)));
    
    std::cout << "yaw of prius" << current_yaw_ << std::endl;
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(current_yaw_);
    nav_msgs::Odometry odom;
    odom_.pose.pose.position.x = centre(0);
    odom_.pose.pose.position.y = centre(1);
    odom_.pose.pose.position.z = centre(2);
    odom_.pose.pose.orientation = odom_quat;
    // odom_pub_.publish(odom);
}

void LocalizationNode::getVelocityOfPrius(Eigen::Vector3d& currentPositionOfPrius) {
    current_time_ = ros::Time::now().toSec();

    current_position_ = currentPositionOfPrius;
    geometry_msgs::Vector3 linearVelOfPrius;

    linearVelOfPrius.x = (current_position_(0) - previous_position_(0)) / (current_time_ - previous_time_);
    linearVelOfPrius.y = (current_position_(1) - previous_position_(1)) / (current_time_ - previous_time_);
    linearVelOfPrius.z = (current_position_(2) - previous_position_(2)) / (current_time_ - previous_time_);
    
    prius_vel_.linear = linearVelOfPrius;
    prius_vel_.angular.z = (current_yaw_ - previous_yaw_) / (current_time_ - previous_time_);
    odom_.twist.twist.linear = linearVelOfPrius;
    vel_pub_.publish(prius_vel_);
    odom_pub_.publish(odom_);

    // publishSetPoint();
    publishVelocity();
    previous_position_ = current_position_;
    previous_time_ = current_time_;
    previous_yaw_ = current_yaw_;
}

void LocalizationNode::publishSetPoint() {
    geometry_msgs::PoseStamped set_pt;
    set_pt.pose = odom_.pose.pose;
    set_pt.pose.position.z = 18;
    setpt_pub_.publish(set_pt);
}

void LocalizationNode::publishVelocity() {
    int k = 7;
    geometry_msgs::TwistStamped drone_vel;

    drone_vel.twist.linear.x = k*(odom_.pose.pose.position.x - odom_drone_.pose.pose.position.x) + prius_vel_.linear.x;
    drone_vel.twist.linear.y = k*(odom_.pose.pose.position.y - odom_drone_.pose.pose.position.y) + prius_vel_.linear.y;
    drone_vel.twist.angular.z = prius_vel_.angular.z + k*(current_yaw_ - drone_yaw_);


    drone_vel_pub_.publish(drone_vel);

}
