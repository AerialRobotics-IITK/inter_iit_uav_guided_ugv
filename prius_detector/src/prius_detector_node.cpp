#include <prius_detector/prius_detector.hpp>

using namespace interiit22::prius_detection;

int main(int argc, char** argv) {
    ros::init(argc, argv, "prius_detector_node");
    ros::NodeHandle nh;

    PriusDetectorNode detect;

    detect.init(nh);
    ros::Rate loop_rate(20);

    while (ros::ok()) {
        detect.run();
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}