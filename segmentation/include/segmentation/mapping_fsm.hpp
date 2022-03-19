#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <ros/console.h>
#include <ros/ros.h>
#include <segmentation/drone_way.h>
#include <vector>
#include <tf/tf.h>
#include <nav_msgs/Odometry.h>

#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Quaternion.h>
#include <tf/transform_datatypes.h>
// #include <LinearMath/btMatrix3x3.h>

namespace mapping_fsm {
class MappingFSM {
    ros::NodeHandle nh_;
    ros::Subscriber way_sub_;
    ros::Publisher way_pub_;
    ros::Subscriber odom_sub_;

  public:
    MappingFSM();
    void wayCallback(const segmentation::drone_way& msg);
    void odomCallback(const nav_msgs::Odometry& msg);
    void convertFrames(const segmentation::drone_way& msg, Eigen::Vector3d& possible_obj);
    void arrayToMatrixConversion();
    int pause_count_;
    bool close_to_obj_;
    Eigen::Vector3d coord_drone_;
    Eigen::Vector3d current_obj_;
    Eigen::Vector3d possible_obj_;
    Eigen::Quaterniond quaternion_drone_;
    std::vector<Eigen::Vector3d> road_mean_path_;

    Eigen::Matrix3d cameraMatrix, invCameraMatrix;
		Eigen::Matrix3d cameraToQuadMatrix;
		Eigen::Matrix3d quadOrientationMatrix, scaleUpMatrix;
		Eigen::Vector3d translation_, camera_translation_vector_;

    std::vector<float> camera_to_quad_matrix_, camera_translation_,camera_matrix_;


    // isMoving()
    // vector of waypoints?
};
}  // namespace mapping_fsm