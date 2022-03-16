#include <offboard/offboard.hpp>

using namespace interiit22::offboard;

int main(int argc, char** argv) {
    ros::init(argc, argv, "explorer_node");
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");

    OffboardNode explore;

    explore.init(nh, nh_private);

    ros::Rate loop_rate(20);

    while (ros::ok()) {
        explore.run();
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
