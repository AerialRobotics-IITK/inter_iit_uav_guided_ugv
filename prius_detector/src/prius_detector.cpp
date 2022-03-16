#include <prius_detector/prius_detector.hpp>

namespace interiit22::prius_detection {

void PriusDetectorNode::init(ros::NodeHandle& nh) {
    int h_min, s_min, v_min;
    int h_max, s_max, v_max;
    int canny_lower, canny_upper, canny_ker;
    int min_contour_area;

    img_sub_ = nh.subscribe("/depth_camera/depth/image_raw", 1, &PriusDetectorNode::imageCallback, this);

    ros::NodeHandle nh_private("~");

    nh_private.getParam("h_min", h_min);
    nh_private.getParam("s_min", s_min);
    nh_private.getParam("v_min", v_min);
    nh_private.getParam("h_max", h_max);
    nh_private.getParam("s_max", s_max);
    nh_private.getParam("v_max", v_max);
    nh_private.getParam("canny_lower", canny_lower);
    nh_private.getParam("canny_upper", canny_upper);
    nh_private.getParam("canny_ker", canny_ker);
    nh_private.getParam("min_contour_area", min_contour_area);

    detect_.setHSVMin(h_min, s_min, v_min);
    detect_.setHSVMax(h_max, s_max, v_max);
    detect_.setCannyParams(canny_lower, canny_upper, canny_ker);
    detect_.setMinArea(min_contour_area);

    centre_pub_ = nh_private.advertise<detector_msgs::Centre>("centre_coord", 10);
    thresh_pub_ = nh_private.advertise<sensor_msgs::Image>("thresh_img", 10);
    contour_pub_ = nh_private.advertise<sensor_msgs::Image>("contours", 10);
    corners_pub_ = nh_private.advertise<detector_msgs::Corners>("corners", 10);
}

void PriusDetectorNode::run() {
    if (img_.empty()) {
        return;
    }

    detect_.thresholdImage(img_);
    detect_.findGoodContours();
    detect_.drawContours(img_);
    detect_.fitRect(img_);

    std::pair<int, int> centre_pair = detect_.getCentre();
    double distance = detect_.getDistance();
    double area = detect_.getArea();
    centre_coord_.x = centre_pair.first;
    centre_coord_.y = centre_pair.second;
    centre_coord_.d = (float) distance;
    centre_coord_.a = (float) area;
    centre_coord_.header.stamp = ros::Time::now();

    cv::Point2f* corners = detect_.getCorners();
    rect_corners_.c1_x = corners[0].x;
    rect_corners_.c1_y = corners[0].y;
    rect_corners_.c2_x = corners[1].x;
    rect_corners_.c2_y = corners[1].y;
    rect_corners_.c3_x = corners[2].x;
    rect_corners_.c3_y = corners[2].y;
    rect_corners_.c4_x = corners[3].x;
    rect_corners_.c4_y = corners[3].y;

    sensor_msgs::ImagePtr thresh_msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", detect_.getThresh()).toImageMsg();
    sensor_msgs::ImagePtr contour_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", img_).toImageMsg();

    thresh_pub_.publish(thresh_msg);
    contour_pub_.publish(contour_msg);
    centre_pub_.publish(centre_coord_);
    corners_pub_.publish(rect_corners_);
}

void PriusDetectorNode::imageCallback(const sensor_msgs::ImageConstPtr& msg) {
    
    uint8_t img[msg->height][msg->width];
    for(int i = 0; i <msg->height; i++) {
        for(int j = 0; j <msg->width; j++) {
            img[i][j] = (int)msg->data[j + msg->width*i];
        }
    }
    cv::Mat src = cv::Mat(msg->height,msg->width,CV_8U,&img);
    img_ = src;
}

}  // namespace interiit22::prius_detection