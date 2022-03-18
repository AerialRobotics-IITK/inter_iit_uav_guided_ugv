#include <prius_detector/prius_localization.hpp>


void LocalizationNode::init(ros::NodeHandle &nh) {
 
  odom_sub_ = nh.subscribe("odom", 1, &LocalizationNode::odomCallback, this);

  // set parameters
  nh.getParam("camera_matrix", camera_matrix_);
  nh.getParam("cam_to_quad_rot", camera_to_quad_matrix_);
  nh.getParam("t_cam", camera_translation_);

  arrayToMatrixConversion();
  debug_ = true;
}

void LocalizationNode::arrayToMatrixConversion() {
  for (int i = 0; i < 3; i++) {
    camera_translation_vector_(i) = camera_translation_[i];
    for (int j = 0; j < 3; j++) {
      cameraToQuadMatrix(i, j) = camera_to_quad_matrix_[3 * i + j];
      cameraMatrix(i,j) = camera_matrix_[3 * i + j];
    }
  }

  invCameraMatrix = cameraMatrix.inverse();
}

void LocalizationNode::odomCallback(const nav_msgs::Odometry& msg) {
  odom_ = msg;
  tf::Quaternion q(msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,
                   msg.pose.pose.orientation.z, msg.pose.pose.orientation.w);
  Eigen::Quaterniond quat = Eigen::Quaterniond(q.w(), q.x(), q.y(), q.z());
  quadOrientationMatrix = quat.normalized().toRotationMatrix().inverse();
  translation_ =
      Eigen::Vector3d(msg.pose.pose.position.x, msg.pose.pose.position.y,
                      msg.pose.pose.position.z);
}

Eigen::Vector3d LocalizationNode::inMapFrame(Eigen::Vector3d& point) {

    scaleUpMatrix(0,0) = scaleUpMatrix(1,1) = scaleUpMatrix(2,2) = point(2);
    Eigen::Vector3d pixel_coordinates(point(0), point(1), 1);
    Eigen::Vector3d quad_frame_coordinates = cameraToQuadMatrix * scaleUpMatrix * invCameraMatrix * pixel_coordinates + camera_translation_vector_;
    Eigen::Vector3d map_frame_coordinates = quadOrientationMatrix * quad_frame_coordinates + translation_;

    return map_frame_coordinates;
}


