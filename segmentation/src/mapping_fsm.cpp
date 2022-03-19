#include <segmentation/mapping_fsm.hpp>
#define OUT_TOPIC "/mavros/setpoint_position/local"
#define CLOSENESS_PARAM 1.0
#define PAUSE_WAIT_TIME 2
namespace mapping_fsm {

MappingFSM::MappingFSM() {
    std::cout << "CONSTRUCTOR : Started" << std::endl;
    coord_drone_.setZero();
    quaternion_drone_.setIdentity();
    quadOrientationMatrix.setIdentity();
    current_obj_.setZero();
    possible_obj_.setZero();
    translation_.setZero();
    pause_count_ = 0;
    close_to_obj_ = false;

    road_mean_path_.push_back(current_obj_); // First waypoint is origin

    // set parameters
    std::cout << "Getting params" << std::endl;
    nh_.getParam("/camera_matrix", camera_matrix_);
    nh_.getParam("/cam_to_quad_rot", camera_to_quad_matrix_);
    nh_.getParam("/t_cam", camera_translation_);
    std::cout << "Got params" << std::endl;
    arrayToMatrixConversion();
    std::cout << "arrayToMatrixConversion done" << std::endl;

    way_sub_ = nh_.subscribe("/drone_way", 1, &MappingFSM::wayCallback, this);
    way_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(OUT_TOPIC, 1);
    odom_sub_ = nh_.subscribe("/mavros/local_position/odom", 1, &MappingFSM::odomCallback, this);

    std::cout << "CONSTRUCTOR : Done" << std::endl;
}

void MappingFSM::arrayToMatrixConversion() {

    for (int i = 0; i < 3; i++) {
        camera_translation_vector_(i) = camera_translation_[i];
        std::cout << "Conversion" << std::endl;
        for (int j = 0; j < 3; j++) {
            cameraToQuadMatrix(i, j) = camera_to_quad_matrix_[3 * i + j];
            cameraMatrix(i,j) = camera_matrix_[3 * i + j];
        }
    }

    invCameraMatrix = cameraMatrix.inverse();
}

void MappingFSM::convertFrames(const segmentation::drone_way& msg, Eigen::Vector3d& possible_obj) {
    // possible_obj contains the waypoint coordinates in the global frame
    
    Eigen::Vector3d camera_frame_coordinates(msg.x, msg.y, msg.z);
    Eigen::Vector3d quad_frame_coordinates = cameraToQuadMatrix * camera_frame_coordinates + camera_translation_vector_;
    possible_obj = quadOrientationMatrix * quad_frame_coordinates + translation_;
}

void MappingFSM::wayCallback(const segmentation::drone_way& msg) {
    // Get the required waypoint in drone coordinates from the message.
    // The drone's odometry will be stored in 2 of the public fields.
    // Get the waypoint in global coordinates from that and publish on the correct topic.
    geometry_msgs::PoseStamped pose;
    pose.header.frame_id = ".";
    pose.header.stamp = ros::Time::now();
    pose.header.seq = 0;
    convertFrames(msg, possible_obj_);
    pose.pose.position.x = possible_obj_(0);
    pose.pose.position.y = possible_obj_(1);
    pose.pose.position.z = possible_obj_(2);

    current_obj_ = road_mean_path_[road_mean_path_.size() - 1];
    current_obj_(2) += 15.0; // need to be 15m above the road

    std::cout << "Current position         :       " << coord_drone_(0) << ", " << coord_drone_(1) << ", " << coord_drone_(2) <<std::endl;
    std::cout << "Trying to reach waypoint " << road_mean_path_.size() << " : " << current_obj_(0) << ", " << current_obj_(1) << ", " << current_obj_(2) <<std::endl;

    tf::Quaternion quat(quaternion_drone_.x(), quaternion_drone_.y(), quaternion_drone_.z(), quaternion_drone_.w());
    // quat.x = quaternion_drone_.x;
    // quat.y = quaternion_drone_.y;
    // quat.z = quaternion_drone_.z;
    // quat.w = quaternion_drone_.w;
    double roll, pitch, yaw;
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
    yaw = yaw + 0.4*msg.theta;

    close_to_obj_ = (coord_drone_ - current_obj_).norm() < CLOSENESS_PARAM;
    if (close_to_obj_) {
        pause_count_++;
    } else {
        pause_count_ = 0;
    }
    if (pause_count_ == PAUSE_WAIT_TIME) {
        pause_count_ = 0;
        close_to_obj_ = false;
        std::cout << "Reached waypoint " << road_mean_path_.size() << " : " << current_obj_(0) << ", " << current_obj_(1) << ", " << current_obj_(2) <<std::endl;
        std::cout << "[FSM] Next Waypoint received." << std::endl;
        road_mean_path_.push_back(possible_obj_);
        way_pub_.publish(pose);
        return;
    }
    // Before publishing set current objective to the waypoint.

    quat.setRPY(roll, pitch, yaw);
    // Publish the earlier objective otherwise
    pose.pose.position.x = current_obj_(0);
    pose.pose.position.y = current_obj_(1);
    pose.pose.position.z = current_obj_(2);
    pose.pose.orientation.x = double(quat.x());
    pose.pose.orientation.y = double(quat.y());
    pose.pose.orientation.z = double(quat.z());
    pose.pose.orientation.w = double(quat.w());
    way_pub_.publish(pose);
    std::cout << "---------------------------------------------" << std::endl;
    return;
}

void MappingFSM::odomCallback(const nav_msgs::Odometry& msg) {
    coord_drone_(0) = msg.pose.pose.position.x;
    coord_drone_(1) = msg.pose.pose.position.y;
    coord_drone_(2) = msg.pose.pose.position.z;
    quaternion_drone_.x() = msg.pose.pose.orientation.x;
    quaternion_drone_.y() = msg.pose.pose.orientation.y;
    quaternion_drone_.z() = msg.pose.pose.orientation.z;
    quaternion_drone_.w() = msg.pose.pose.orientation.w;


    tf::Quaternion q(msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,
                   msg.pose.pose.orientation.z, msg.pose.pose.orientation.w);
    Eigen::Quaterniond quat = Eigen::Quaterniond(q.w(), q.x(), q.y(), q.z());
    quadOrientationMatrix = quat.normalized().toRotationMatrix().inverse();
    translation_ = Eigen::Vector3d(msg.pose.pose.position.x, msg.pose.pose.position.y,
                      msg.pose.pose.position.z);
}

}  // namespace mapping_fsm
