#include <prius_detector/prius_localization.hpp>

#include <cv_bridge/cv_bridge.h>
#include <detector_msgs/Centre.h>
#include <detector_msgs/Corners.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <bits/stdc++.h>
#include <iostream>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <utility>
#include <vector>

#include <pcl_ros/transforms.h>
#include <pcl/conversions.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/point_cloud.h>
#include <sensor_msgs/PointCloud2.h>


LocalizationNode localization;

pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_{new pcl::PointCloud<pcl::PointXYZRGB>};
pcl::PCLPointCloud2 pcl_pc2;


cv::Mat img_;
cv::Mat depth;
bool flag = false;
cv::Scalar color = cv::Scalar(255, 0, 0);

Eigen::Vector3d inCameraFrame(float& x, float& y) {

    Eigen::Vector3d point;
    point(0) = (*cloud_).at(x, y).x;
    point(1) = (*cloud_).at(x, y).y;
    point(2) = (*cloud_).at(x, y).z;

    return point;
}
void imageProcessing() {
    if (!flag) return;

    int maxval;
    int thres;

    ros::NodeHandle nh;

    nh.getParam("thres", thres);
    nh.getParam("maxval", maxval);

	cv::Mat mask;
    int subt = depth.at<float>(320,480);
    cv::Scalar max_white = cv::Scalar(120-subt*1.5, 120-subt*1.5,120-subt*1.5);
    cv::inRange(img_,(0,0,0),max_white,mask);
    // cv::threshold(img_, mask, thres, maxval,CV_THRESH_BINARY_INV);

    // Finding contours..
    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours(mask,contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);

    int car_hull = 0;
    cv::Mat drawing = cv::Mat::zeros( mask.size(), CV_8UC1 );
    std::vector< std::vector<cv::Point> > hull(contours.size());
    if(contours.size() == 0){
        std::cout << "No contour found..\n";
        return;
    }
    for(int i = 0; i < contours.size(); i++){
        cv::convexHull(cv::Mat(contours[i]), hull[i], false);
        std::cout<<"hull "<<i<<" : "<<hull[i].size();
        if(hull.size()==0){
            std::cout << "No hull found..\n";
            return;
        }
        else if(20<hull[i].size()&&hull[i].size()<40){
            car_hull = i;
            cv::drawContours( drawing, hull, i, color, 1, 8 );
        }
    }
    
    cv::drawContours( drawing, contours, -1, color, 1, 8);

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

    fcenter.x /= 2;
    fcenter.y /= 2;
    
    std::cout<<"x : "<<center.x<<" y : "<<center.y<<"\n";
    std::cout<<"x : "<<fcenter.x<<" y : "<<fcenter.y<<"\n";

    std::cout<<"depth : "<<depth.at<float>(center.x,center.y)<<"\n";

    Eigen::Vector3d prius_center = inCameraFrame(center.x, center.y);
    Eigen::Vector3d front_center = inCameraFrame(fcenter.x, fcenter.y); 

    std::cout << "camera frame prius: " << prius_center(0) << " " << prius_center(1) << " "<<prius_center(2)<< std::endl;

    Eigen::Vector3d prius_center_global = localization.inMapFrame(prius_center);
    Eigen::Vector3d front_center_global = localization.inMapFrame(front_center);

    std::cout << "global frame coordinates" <<prius_center_global(0) << " " << prius_center_global(1) << " " << prius_center_global(2) << std::endl;

    cv::circle(mask, center, 2, cv::Scalar(0,255,0), 2);
    cv::circle(mask, fcenter, 2, cv::Scalar(0,0,255), 2);

    localization.getOrientationOfPrius(front_center_global, prius_center_global);

    // Display
    cv::imshow("Original", mask);
    cv::imshow("Center Detection", drawing);
    cv::waitKey(1);

    return;
}

void imageCallback(const sensor_msgs::ImageConstPtr& msg) {

	cv_bridge::CvImagePtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1);
    } 
	
	catch (cv_bridge::Exception& e) {
        ROS_ERROR("tutorialROSOpenCV::main.cpp::cv_bridge exception: %s", e.what());
        return;
    }

    img_ = cv_ptr->image;

    // !img_ is Matrix: 32FC1 640x480 
    img_.copyTo(depth);
    // std::cout<<"depth : "<<img_.at<float>(center.x,center.y)<<"\n";

    cv::normalize(img_, img_, 0, 255, cv::NORM_MINMAX);
    cv::convertScaleAbs(img_, img_, 1, 0.0);
	cv::namedWindow("img_");
    cv::imshow("img_", img_);
	cv::waitKey(1);
    flag = true;
}

void pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& input) {
    pcl_conversions::toPCL(*input, pcl_pc2);
    pcl::fromPCLPointCloud2(pcl_pc2, *cloud_);
}

int main(int argc, char** argv) {

	ros::init(argc, argv, "prius_detector_node");
    ros::NodeHandle nh;

	ros::Subscriber img_sub_ = nh.subscribe("/depth_camera/depth/image_raw", 10,imageCallback);
	ros::Publisher contour_pub_ = nh.advertise<sensor_msgs::Image>("/contours", 10);
    ros::Subscriber point_cloud_ = nh.subscribe("/depth_camera/depth/points", 10, pointCloudCallback);

    localization.init(nh);

	ros::Rate loop_rate(20);
	while (ros::ok()) {
		ros::spinOnce();
		loop_rate.sleep();
        imageProcessing();
	}
}