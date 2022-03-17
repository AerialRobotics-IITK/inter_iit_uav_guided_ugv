#pragma once

#include <iostream>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <utility>
#include <vector>

namespace interiit22::prius_detection {

class PriusDetector {
  public:
    std::pair<int, int> getCentre();
    cv::Mat getThresh();
    cv::Point2f* getCorners();
    double getDistance();
    double getArea();

    void setHSVMin(const int& h, const int& s, const int& v);
    void setHSVMax(const int& h, const int& s, const int& v);
    void setMinArea(const int& area);
    void setCannyParams(const int& lower, const int& upper, const int& size);

    void thresholdImage(cv::Mat& img);
    void findGoodContours();
    void drawContours(cv::Mat& img);
    void findFrameCentre(cv::Mat& img);
    void fitRect(cv::Mat& img);

    static constexpr double scale_factor = 20160;  // heuristically determined

  private:
    std::pair<int, int> centre_;

    cv::Scalar hsv_min_;
    cv::Scalar hsv_max_;

    cv::Mat thresh_img_;

    cv::Point2f center_;
    cv::Point2f rect_points[4];

    std::vector<std::vector<cv::Point>> good_contours_;

    int canny_param_lower_;
    int canny_param_upper_;
    int canny_kernel_size_;

    double min_contour_area_;
    double distance_;
    double area_;
    int threshold_min_,threshold_max_;
};

}  // namespace interiit22::prius_detection