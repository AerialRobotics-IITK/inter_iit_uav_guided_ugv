#include <explorer/explorer.hpp>

using namespace interiit21::explorer;

int main(int argc, char** argv) {
    ros::init(argc, argv, "explorer_node");
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");

    ExplorerNode explore;

    explore.init(nh, nh_private);

    ros::Rate loop_rate(20);

    while (ros::ok()) {
        explore.run();
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
