#include <ros/console.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <sensor_msgs/image_encodings.h>

#include <segmentation/drone_way.h>

#include <map>
#include <math.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/ground_plane_comparator.h>
#include <pcl/segmentation/organized_connected_component_segmentation.h>
#include <pcl/segmentation/organized_multi_plane_segmentation.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>
#include <random>
#include <set>
#include <utility>
#include <vector>

namespace road_detector {

typedef pcl::PointXYZRGB PointT;
class RoadDetector {
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    image_transport::Publisher image_pub_;
    ros::Publisher cloud_pub_;
    ros::Subscriber cloud_sub_;
    ros::Publisher waypoint_pub_;

  public:
    const std::string OPENCV_WINDOW = "Image window";
    // std::vector<cv::Mat> img_=std::vector<cv::Mat>{3, cv::Mat(480, 640, CV_8UC3, cv::Scalar(0,0, 0))};
    cv::Mat img_[3];
    cv::Point waypoint;
    float fx, cx, fy, cy;
    segmentation::drone_way next_waypoint_;

    float computeAlignment(pcl::PointCloud<PointT>::Ptr& cloud, pcl::PointIndices::Ptr inliers);
    float computeDeltaZ(pcl::PointCloud<PointT>::Ptr& cloud, pcl::PointIndices::Ptr inliers, float fraction);
    RoadDetector();
    void makeImageBlack(int i);
    void imageFromPointcloud(pcl::PointCloud<PointT>::Ptr cloud, pcl::PointIndices::Ptr inliers, pcl::PointCloud<PointT>::Ptr& label_image, int colour, int i);
    void extractFromInliers(pcl::PointCloud<PointT>::Ptr in, pcl::PointCloud<PointT>::Ptr& out, pcl::PointIndices::Ptr inliers);
    void imageCb(const sensor_msgs::ImageConstPtr& msg);
    void pcCallback(const sensor_msgs::PointCloud2::ConstPtr& input);
    // Mean Path Helper Function
    void drawAxis(cv::Mat&, cv::Point, cv::Point, cv::Scalar, const float);
    double getOrientation(const std::vector<cv::Point>&, cv::Mat&);
    void meanPath();
    // Mean Path Helper Function
    float computeAverageZ(pcl::PointCloud<PointT>::Ptr& cloud, pcl::PointIndices::Ptr inliers, float fraction, int i);
    std::vector<int> randomPick(int N, int k);
    float get_perpendicular_distance(cv::Point p1, cv::Point image_center, cv::Point p2);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_{new pcl::PointCloud<pcl::PointXYZRGB>};
    pcl::PCLPointCloud2 pcl_pc2;

    pcl::ModelCoefficients::Ptr coefficients{new pcl::ModelCoefficients};
    pcl::PointIndices::Ptr inliers{new pcl::PointIndices};
    pcl::SACSegmentation<pcl::PointXYZRGB> seg;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr label_image{new pcl::PointCloud<pcl::PointXYZRGB>};
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr next{new pcl::PointCloud<pcl::PointXYZRGB>};
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr next2{new pcl::PointCloud<pcl::PointXYZRGB>};
    std::vector<std::pair<float, int>> alignment;
    std::map<int, float> averageZ;
    float prevZ;
    int min_alignment_idx;

    std::random_device rd;
    std::unordered_set<int> elems;
};

}  // namespace road_detector