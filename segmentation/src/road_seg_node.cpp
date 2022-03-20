#include <segmentation/road_seg.hpp>

int main(int argc, char** argv) {
    ros::init(argc, argv, "road_seg_node");
    road_detector::RoadDetector rd;
    ros::Rate loop_rate(20);
    Eigen::Vector3f nominal_road_normal(0.0, 0.0, 1.0);

    ros::spin();
}