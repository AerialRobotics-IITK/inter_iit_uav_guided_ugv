#include <prius_detector/prius_detector.hpp>
#include <string>
#include <opencv2/opencv.hpp>

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
    nh_private.getParam("threshold_min", threshold_min_);
    nh_private.getParam("threshold_max", threshold_max_);

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
    sensor_msgs::ImagePtr contour_msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", img_).toImageMsg();

    thresh_pub_.publish(thresh_msg);
    contour_pub_.publish(contour_msg);
    centre_pub_.publish(centre_coord_);
    corners_pub_.publish(rect_corners_);
}

void PriusDetectorNode::imageCallback(const sensor_msgs::ImageConstPtr& msg) {

    
    // int h = msg->height;
    // int w = msg->width*4;
    // uint8_t img[h][w];
    // for(int i = 0; i <h; i++) {
    //     for(int j = 0; j <w; j++) {
    //         img[i][j] = (int)msg->data[j + w*i];
    //     }
    // }
    // cv::Mat src = cv::Mat(h,w/4,CV_8UC1,&img);
    cv_bridge::CvImagePtr cv_ptr;
    // Convert from the ROS image message to a CvImage suitable for working with OpenCV for processing
    try {
        // Always copy, returning a mutable CvImage
        // OpenCV expects color images to use BGR channel order.
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1);
    } catch (cv_bridge::Exception& e) {
        // if there is an error during conversion, display it
        ROS_ERROR("tutorialROSOpenCV::main.cpp::cv_bridge exception: %s", e.what());
        return;
    }

    // Copy the image.data to imageBuf.
    img_ = cv_ptr->image;
    // cv::Mat temp_img = cv_ptr->image;
    // cv::cvtColor(temp_img, img_, cv::COLOR_GRAY2RGB);
    cv::normalize(img_, img_, 0, 255, cv::NORM_MINMAX);
    cv::convertScaleAbs(img_, img_, 1, 0.0);
    cv::imshow("first draw img_", img_);


    //! compute mask (you could use a simple threshold if the image is always as good as the one you provided)

    cv::Mat mask;
    cv::threshold(img_, mask, threshold_min_, threshold_max_, CV_THRESH_BINARY_INV | CV_THRESH_OTSU);

    //! Finding contours..
    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours(mask,contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);

    //! Draw contours and find biggest contour (if there are other contours in the image, assuming the biggest one is the desired rect)
    int biggestContourIdx = -1;
    float biggestContourArea = 0;
    cv::Mat drawing = cv::Mat::zeros( mask.size(), CV_8UC3 );
    cv::imshow("second draw drawing", drawing);


    for (int i = 0; i< contours.size(); i++){
        cv::Scalar color = cv::Scalar(0, 100, 0);
        drawContours( drawing, contours, i, color, 1, 8, hierarchy, 0, cv::Point() );

        float ctArea= cv::contourArea(contours[i]);
        if (ctArea > biggestContourArea){
            biggestContourArea = ctArea;
            biggestContourIdx = i;
        }
    }

    //! If no contour found
    if(biggestContourIdx < 0){
        std::cout << "No contour found..\n";
        return;
    }

    //! compute the rotated bounding rect of the biggest contour! (this is the part that does what you want/need)
    cv::RotatedRect boundingBox = cv::minAreaRect(contours[biggestContourIdx]);

    //one thing to remark: this will compute the OUTER boundary box, so maybe you have to erode/dilate if you want something between the ragged lines

    //! Draw the rotated rect
    cv::Point2f corners[4];
    boundingBox.points(corners);
    cv::line(drawing, corners[0], corners[1], cv::Scalar(255,255,255));
    cv::line(drawing, corners[1], corners[2], cv::Scalar(255,255,255));
    cv::line(drawing, corners[2], corners[3], cv::Scalar(255,255,255));
    cv::line(drawing, corners[3], corners[0], cv::Scalar(255,255,255));
    
    //! Draw center point..
    cv::Point2f center;
    for (int i=0; i<4; i++){
        center.x += corners[i].x;
        center.y += corners[i].y;
    } 
    center.x /= 4;
    center.y /= 4;
    cv::circle(drawing, center, 2, cv::Scalar(255,255,255), 2);

    //! Display
    cv::imshow("Original", img_);
    cv::imshow("Center Detection", drawing);
    cv::waitKey(0);
    cv::destroyAllWindows();
    return;
}

}  // namespace interiit22::prius_detection