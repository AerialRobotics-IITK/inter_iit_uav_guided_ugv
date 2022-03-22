#include <segmentation/mapping_fsm.hpp>

int main(int argc, char** argv) {
    ros::init(argc, argv, "drone_waypoint_node");
    mapping_fsm::MappingFSM fsm;
    ros::Rate loop_rate(40);

    ros::spin();
}