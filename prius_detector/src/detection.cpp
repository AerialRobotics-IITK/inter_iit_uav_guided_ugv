#include <cv_bridge/cv_bridge.h>
#include <detector_msgs/Centre.h>
#include <detector_msgs/Corners.h>
#include <image_transport/image_transport.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <bits/stdc++.h>
#include <iostream>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <utility>
#include <vector>

using namespace std;
using namespace cv;

cv::Mat img_;
cv::Mat depth(img_.size(), img_.type());

// !for debuging
std::string type2str(int type) {
  std::string r;

  uchar depth = type & CV_MAT_DEPTH_MASK;
  uchar chans = 1 + (type >> CV_CN_SHIFT);

  switch ( depth ) {
    case CV_8U:  r = "8U"; break;
    case CV_8S:  r = "8S"; break;
    case CV_16U: r = "16U"; break;
    case CV_16S: r = "16S"; break;
    case CV_32S: r = "32S"; break;
    case CV_32F: r = "32F"; break;
    case CV_64F: r = "64F"; break;
    default:     r = "User"; break;
  }

  r += "C";
  r += (chans+'0');

  return r;
}

cv::Scalar color = cv::Scalar(255, 0, 0);
cv::Mat GetPixelsFromMat( const cv::Mat& I, const std::vector<cv::Point2f>& points )
{
    // some pre-condition:
    cv::Mat res( 1, (int)points.size( ), CV_8UC1 );

    int i = 0;
    for( const auto& point : points )
        res.ptr( 0 )[ i++ ] = I.at<uchar>( cvRound( point.y ), cvRound( point.x ) );

    return res;
}

void imageProcessing(cv::Mat &depth) {
// void imageProcessing() {
    ros::NodeHandle nh;
    int maxval;
    int hull_param;
    int thres;

    nh.getParam("hull_param", hull_param);
    nh.getParam("thres", thres);
    nh.getParam("maxval", maxval);

	cv::Mat mask;
    cv::inRange(img_,(0,0,0),(120,120,120),mask);
    // cv::threshold(img_, mask, thres, maxval,CV_THRESH_BINARY_INV);

    // Finding contours..
    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours(mask,contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);

    int car_hull = 0;
    cv::Mat drawing = cv::Mat::zeros( mask.size(), CV_8UC1 );
    vector< vector<Point> > hull(contours.size());
    if(contours.size() == 0){
        std::cout << "No contour found..\n";
        return;
    }
    for(int i = 0; i < contours.size(); i++){
        convexHull(Mat(contours[i]), hull[i], false);
        std::cout<<"hull "<<i<<" : "<<hull[i].size();
        if(hull.size()==0){
            std::cout << "No hull found..\n";
            return;
        }
        else if(15<hull[i].size()&&hull[i].size()<40){
            car_hull = i;
            drawContours( drawing, hull, i, color, 1, 8 );
        }
    }
    // drawContours( drawing, hull, 1, color, 1, 8 );
    // Draw contours and find biggest contour (if there are other contours in the image, assuming the biggest one is the desired rect)
    // int biggestContourIdx = -1;
    // float biggestContourArea = 0;
    // cv::imshow("second draw drawing", drawing);

    
    drawContours( drawing, contours, -1, color, 1, 8);
    // for (int i = 0; i< contours.size(); i++){
    //     float ctArea= cv::contourArea(contours[i]);
    //     std::cout<<"Area of countour : "<<ctArea<<"\n";
    //     if (ctArea > biggestContourArea){
    //         biggestContourArea = ctArea;
    //         biggestContourIdx = i;
    //     }
    // }
    std::cout<<"--------------------------------\n";

    // If no contour found

    // compute the rotated bounding rect of the biggest contour! (this is the part that does what you want/need)
    
    cv::RotatedRect boundingBox = cv::minAreaRect(contours[car_hull]);

    //one thing to remark: this will compute the OUTER boundary box, so maybe you have to erode/dilate if you want something between the ragged lines

    // Draw the rotated rect
    cv::Point2f corners[4];

    // Extracting corners of the bounding box
    boundingBox.points(corners);
    cv::line(mask, corners[0], corners[1], cv::Scalar(255,255,255));
    cv::line(mask, corners[1], corners[2], cv::Scalar(255,255,255));
    cv::line(mask, corners[2], corners[3], cv::Scalar(255,255,255));
    cv::line(mask, corners[3], corners[0], cv::Scalar(255,255,255));
    
    // Draw center point..
    cv::Point2f center;
    cv::Point2f fcenter;

    for (int i=0; i<4; i++){
        center.x += corners[i].x;
        center.y += corners[i].y;
    } 
    for (int i=0; i<2; i++){
        fcenter.x += corners[i].x;
        fcenter.y += corners[i].y;
    } 

    center.x /= 4;
    center.y /= 4;

    center.x /= 2;
    center.y /= 2;
    std::cout<<"x : "<<center.x<<" y : "<<center.y<<"\n";
    std::cout<<"depth : "<<depth.at<float>(center.x,center.y)<<"\n";

    // if(depth.at<float>(center.x,center.y)>10){
    //     thres_max = 100;
    // }
    
    cv::circle(mask, center, 2, cv::Scalar(0,255,0), 2);
    cv::circle(mask, fcenter, 2, cv::Scalar(0,0,255), 2);

    // Display
    cv::imshow("Original", mask);
    cv::imshow("Center Detection", drawing);
    cv::waitKey(1);

    // cv::destroyAllWindows(); 
    return;
}

void imageCallback(const sensor_msgs::ImageConstPtr& msg) {

	cv_bridge::CvImagePtr cv_ptr;
    // Convert from the ROS image message to a CvImage suitable for working with OpenCV for processing
    try {
        // Always copy, returning a mutable CvImage
        // OpenCV expects color images to use BGR channel order.
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1);
    } 
	
	catch (cv_bridge::Exception& e) {
        // if there is an error during conversion, display it
        ROS_ERROR("tutorialROSOpenCV::main.cpp::cv_bridge exception: %s", e.what());
        return;
    }

    // Copy the image.data to imageBuf.
    img_ = cv_ptr->image;
    // cv::Mat temp_img = cv_ptr->image;
    // cv::cvtColor(temp_img, img_, cv::COLOR_GRAY2RGB);
    // std::cout<<"img_"<<img_<<"\n";

    // !img_ is Matrix: 32FC1 640x480 
    img_.copyTo(depth);
    // std::cout<<"depth : "<<img_.at<float>(center.x,center.y)<<"\n";

    // std::cout<<"depth : "<<depth<<std::endl;
    cv::normalize(img_, img_, 0, 255, cv::NORM_MINMAX);
    cv::convertScaleAbs(img_, img_, 1, 0.0);
	cv::namedWindow("img_");
    cv::imshow("img_", img_);
	cv::waitKey(1);
    imageProcessing(depth);
    // imageProcessing();


}

int main(int argc, char** argv) {

	ros::init(argc, argv, "prius_detector_node");
    ros::NodeHandle nh;

	ros::Subscriber img_sub_ = nh.subscribe("/depth_camera/depth/image_raw", 10,imageCallback);
	ros::Publisher contour_pub_ = nh.advertise<sensor_msgs::Image>("contours", 10);

	ros::Rate loop_rate(20);
	while (ros::ok()) {
		ros::spinOnce();
		// imageProcessing();
		loop_rate.sleep();
	}

}