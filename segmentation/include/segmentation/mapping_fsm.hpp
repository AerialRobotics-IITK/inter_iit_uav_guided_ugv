#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <ros/console.h>
#include <ros/ros.h>
#include <segmentation/drone_way.h>
#include <vector>
namespace mapping_fsm {
class MappingFSM {
    ros::NodeHandle nh_;
    ros::Subscriber way_sub_;
    ros::Publisher way_pub_;
    ros::Subscriber odom_sub_;

  public:
    MappingFSM();
    void wayCallback(const segmentation::drone_way& msg);
    void odomCallback(const geometry_msgs::Pose& msg);
    void convertFrames(const segmentation::drone_way& msg, Eigen::Vector3f& possible_obj);
    int pause_count_;
    bool close_to_obj_;
    Eigen::Vector3f coord_drone_;
    Eigen::Vector3f current_obj_;
    Eigen::Vector3f possible_obj_;
    Eigen::Quaternionf quaternion_drone_;
    std::vector<Eigen::Vector3f> road_mean_path_;
    // isMoving()
    // vector of waypoints?
};
}  // namespace mapping_fsm