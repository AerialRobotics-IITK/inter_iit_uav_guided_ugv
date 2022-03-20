#include <prius_detector/prius_localization.hpp>

void LocalizationNode::init(ros::NodeHandle& nh) {
    odom_sub_ = nh.subscribe("/mavros/local_position/odom", 1, &LocalizationNode::odomCallback, this);
    odom_pub_ = nh.advertise<nav_msgs::Odometry>("/prius_odom", 50);
    vel_pub_ = nh.advertise<geometry_msgs::Twist>("/prius_vel", 50);

    // set parameters
    nh.getParam("/camera_matrix", camera_matrix_);
    nh.getParam("/cam_to_quad_rot", camera_to_quad_matrix_);
    nh.getParam("/t_cam", camera_translation_);

    arrayToMatrixConversion();
    debug_ = true;
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
    odom_ = msg;
    tf::Quaternion q(msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w);
    Eigen::Quaterniond quat = Eigen::Quaterniond(q.w(), q.x(), q.y(), q.z());
    quadOrientationMatrix = quat.normalized().toRotationMatrix().inverse();
    translation_ = Eigen::Vector3d(msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z);
}

Eigen::Vector3d LocalizationNode::inMapFrame(Eigen::Vector3d& point) {
    Eigen::Vector3d quad_frame_coordinates = cameraToQuadMatrix * point + camera_translation_vector_;
    Eigen::Vector3d map_frame_coordinates = quadOrientationMatrix * quad_frame_coordinates + translation_;

    return map_frame_coordinates;
}

// float LocalizationNode::getOrientationOfPrius(std::vector<std::pair<float, float>>& corners, std::pair<float, float> centre) {
void LocalizationNode::getOrientationOfPrius(Eigen::Vector3d& midPointOfFrontWheels, Eigen::Vector3d& centre) {
    float yaw;
    /*
      The vector corners stores all the four corners of the prius. The corners[0] stores the coordinates of the left front
      wheel(the first entry is x coordinate and the second entry is the y coordinate). The corners[1], corner[2] and
      corners[3] stores the coordinates of the front right, back left and back right wheel respectively. However, the
      coordinates of front wheels are needed only for this function to work properly.
    */
    // float left_horizontal = abs(centre.first - corners[0].first);
    // float left_vertical = abs(centre.second - corners[0].second);
    // float alpha = atan(left_vertical / (1.0 * left_horizontal));

    // float right_horizontal = abs(centre.first - corners[1].first);
    // float right_vertical = abs(centre.second - corners[1].second);
    // float beta = atan(right_vertical / (1.0 * right_horizontal));
    // if (alpha <= beta) {
    //     // The vehicle needs to move rightwards.
    //     float front_slope = atan(((1.0) * (corners[0].second - corners[1].second)) / ((1.0) * (corners[0].first - corners[1].first)));
    //     front_slope = abs(front_slope);
    //     if (front_slope < M_PI / 2.0) {
    //         front_slope = M_PI - front_slope;
    //     }
    //     yaw = M_PI - front_slope;
    //     return yaw;
    // }
    // // Else the vehicle needs to move leftwards.
    // float front_slope = atan(((1.0) * (corners[0].second - corners[1].second)) / ((1.0) * (corners[0].first - corners[1].first)));
    // front_slope = abs(front_slope);
    // if (front_slope > M_PI / 2.0) {
    //     front_slope = M_PI - front_slope;
    // }
    // yaw = front_slope;
    // return yaw;
    yaw = atan((midPointOfFrontWheels[1] - centre[1]) / (midPointOfFrontWheels[0] - centre[0]));
    std::cout << "yaw of prius" << yaw << std::endl;
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(yaw);
    odom_.pose.pose.position.x = centre[0];
    odom_.pose.pose.position.y = centre[1];
    odom_.pose.pose.position.z = centre[2];
    odom_.pose.pose.orientation = odom_quat;
    // odom_pub_.publish(odom_);
}

void LocalizationNode::getVelocityOfPrius(Eigen::Vector3d& currentPositionOfPrius, Eigen::Vector3d& previousPositionOfPrius, double previousTime) {
    double currentTime = ros::Time::now().toSec();
    geometry_msgs::Vector3 linearVelOfPrius;
    linearVelOfPrius.x = (currentPositionOfPrius[0] - previousPositionOfPrius[0]) / (currentTime - previousTime);
    linearVelOfPrius.y = (currentPositionOfPrius[1] - previousPositionOfPrius[1]) / (currentTime - previousTime);
    linearVelOfPrius.z = (currentPositionOfPrius[2] - previousPositionOfPrius[2]) / (currentTime - previousTime);
    geometry_msgs::Twist velocityOfPrius;
    velocityOfPrius.linear = linearVelOfPrius;
    odom_.twist.twist.linear = linearVelOfPrius;
    vel_pub_.publish(velocityOfPrius);
    odom_pub_.publish(odom_);
}