#include <segmentation/mapping_fsm.hpp>
#define OUT_TOPIC "/mavros/setpoint_position/local"
#define CLOSENESS_PARAM 0.5
#define PAUSE_WAIT_TIME 2
namespace mapping_fsm {

MappingFSM::MappingFSM() {
    way_sub_ = nh_.subscribe("/drone_way", 1, &MappingFSM::wayCallback, this);
    way_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(OUT_TOPIC, 1);
    odom_sub_ = nh_.subscribe("/mavros/local_position/odom", 1, &MappingFSM::odomCallback, this);
    coord_drone_.setZero();
    quaternion_drone_.setIdentity();
    current_obj_.setZero();
    possible_obj_.setZero();
    pause_count_ = 0;
    close_to_obj_ = false;

    road_mean_path_.push_back(current_obj_); // First waypoint is origin
}

void MappingFSM::convertFrames(const segmentation::drone_way& msg, Eigen::Vector3f& possible_obj) {
    // possible_obj contains the waypoint coordinates in the global frame
    // TODO
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

    close_to_obj_ = (coord_drone_ - current_obj_).norm() < CLOSENESS_PARAM;
    if (close_to_obj_) {
        pause_count_++;
    } else {
        pause_count_ = 0;
    }
    if (pause_count_ == PAUSE_WAIT_TIME) {
        pause_count_ = 0;
        close_to_obj_ = false;
        std::cout << "[FSM] Next Waypoint received." << std::endl;
        road_mean_path_.push_back(possible_obj_);
        way_pub_.publish(pose);
        return;
    }
    // Before publishing set current objective to the waypoint.

    // Publish the earlier objective otherwise
    pose.pose.position.x = current_obj_(0);
    pose.pose.position.y = current_obj_(1);
    pose.pose.position.z = current_obj_(2);
    way_pub_.publish(pose);

    return;
}

void MappingFSM::odomCallback(const geometry_msgs::Pose& msg) {
    coord_drone_(0) = msg.position.x;
    coord_drone_(1) = msg.position.y;
    coord_drone_(2) = msg.position.z;
    quaternion_drone_.x() = msg.orientation.x;
    quaternion_drone_.y() = msg.orientation.y;
    quaternion_drone_.z() = msg.orientation.z;
    quaternion_drone_.w() = msg.orientation.w;
}

}  // namespace mapping_fsm
