#pragma once

#include <cv_bridge/cv_bridge.h>
#include <detector_msgs/Centre.h>
#include <detector_msgs/Corners.h>
#include <image_transport/image_transport.h>
#include <prius_detector/libprius_detection.hpp>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>

namespace interiit22::prius_detection {

class PriusDetectorNode {
  public:
    void init(ros::NodeHandle& nh);
    void run();
    void get_prius_coordinates();

  private:
    void imageCallback(const sensor_msgs::ImageConstPtr& msg);

    cv::Mat img_;

    ros::Subscriber img_sub_;
    ros::Publisher centre_pub_;
    ros::Publisher thresh_pub_;
    ros::Publisher contour_pub_;
    ros::Publisher corners_pub_;

    PriusDetector detect_;

    detector_msgs::Centre centre_coord_;
    detector_msgs::Corners rect_corners_;
};

}  // namespace interiit22::prius_detection