#include <ros/console.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <sensor_msgs/image_encodings.h>

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
#include <vector>
typedef pcl::PointXYZRGB PointT;
int count = 0;
static const std::string OPENCV_WINDOW = "Image window";
float normal_dot_threshold = 0.8;
class ImageConverter {
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    image_transport::Publisher image_pub_;
    ros::Publisher pub;
    ros::Subscriber sub;

  public:
    cv::Mat img_;

    ImageConverter()
        : it_(nh_) {
        // Subscrive to input video feed and publish output video feed
        image_sub_ = it_.subscribe("/camera/color/image_raw", 1, &ImageConverter::imageCb, this);
        image_pub_ = it_.advertise("/image_converter/output_video", 1);

        sub = nh_.subscribe("/camera/depth/points", 1, &ImageConverter::pcCallback, this);
        pub = nh_.advertise<sensor_msgs::PointCloud2>("/processed/pointcloud", 1);

        cv::namedWindow(OPENCV_WINDOW);
    }

    ~ImageConverter() {
        cv::destroyWindow(OPENCV_WINDOW);
    }

    void imageCb(const sensor_msgs::ImageConstPtr& msg) {
        cv_bridge::CvImagePtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        } catch (cv_bridge::Exception& e) {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }

        // Draw an example circle on the video stream
        img_ = cv_ptr->image;
        if (cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60)
            cv::circle(img_, cv::Point(50, 50), 10, CV_RGB(255, 0, 0));

        // Update GUI Window
        // cv::imshow(OPENCV_WINDOW, img_);
        // cv::waitKey(3);

        // Output modified video stream
        image_pub_.publish(cv_ptr->toImageMsg());
    }

    void pcCallback(const sensor_msgs::PointCloud2::ConstPtr& input) {
        // Create a container for the data. --------------------------------------------------------------------------------
        pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
        pcl::PCLPointCloud2 pcl_pc2;
        pcl_conversions::toPCL(*input, pcl_pc2);
        pcl::fromPCLPointCloud2(pcl_pc2, *cloud);

        pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
        pcl::SACSegmentation<pcl::PointXYZRGB> seg;
        seg.setOptimizeCoefficients(true);
        seg.setModelType(pcl::SACMODEL_PLANE);
        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setDistanceThreshold(0.1);

        seg.setInputCloud(cloud);
        seg.segment(*inliers, *coefficients);

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr label_image(new pcl::PointCloud<pcl::PointXYZRGB>);
        if (inliers->indices.size() == 0) {
            std::cout << "Could not estimate a planar model for the given scene" << std::endl;
        }
        std::vector<float> average_normal(3, 0.0);

        pcl::IntegralImageNormalEstimation<PointT, pcl::Normal> ne;
        ne.setNormalEstimationMethod(ne.AVERAGE_3D_GRADIENT);
        ne.setMaxDepthChangeFactor(0.02f);
        ne.setNormalSmoothingSize(10.0f);
        pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
        ne.setInputCloud(cloud);
        ne.compute(*normals);
        // std::cout << "WTF" >> std::endl;

        // [554.254691191187, 0.0, 320.5, 0.0, 554.254691191187, 240.5, 0.0, 0.0, 1.0]
        float fx = 554.254691191187;
        float cx = 320.5;
        float fy = 554.254691191187;
        float cy = 240.5;

        for (int y = 0; y < img_.rows; y++) {
            for (int x = 0; x < img_.cols; x++) {
                img_.at<cv::Vec3b>(y, x)[0] = 0;
                img_.at<cv::Vec3b>(y, x)[1] = 0;
                img_.at<cv::Vec3b>(y, x)[2] = 0;
            }
        }

        for (const auto& idx : inliers->indices) {
            PointT n = cloud->points[idx];
            n.r = 255;
            label_image->points.push_back(n);
            std::cout << (normals->points[idx]).normal_x << " " << (normals->points[idx]).normal_y << " " << (normals->points[idx]).normal_z << std::endl;
            average_normal[0] += (normals->points[idx]).normal_x;
            average_normal[1] += (normals->points[idx]).normal_y;
            average_normal[2] += (normals->points[idx]).normal_z;
            if (n.z != 0) {
                float m1 = n.x / n.z;
                float n1 = n.y / n.z;

                int x = int(m1 * fx + cx);
                int y = int(n1 * fy + cy);

                img_.at<cv::Vec3b>(y, x)[0] = 0;
                img_.at<cv::Vec3b>(y, x)[1] = 255;
                img_.at<cv::Vec3b>(y, x)[2] = 0;
            }
        }
        average_normal[0] /= inliers->indices.size();
        average_normal[1] /= inliers->indices.size();
        average_normal[2] /= inliers->indices.size();

        std::cout << "average normal: " << average_normal[0] << " " << average_normal[1] << " " << average_normal[2] << std::endl;

        sensor_msgs::PointCloud2 cloud_publish;
        pcl::toROSMsg(*label_image, cloud_publish);
        cloud_publish.header = input->header;
        pub.publish(cloud_publish);

        cv::imshow(OPENCV_WINDOW, img_);
        cv::waitKey(3);

        // count += 1;
        // std::cout<<count<<std::endl;
    }
};
int main(int argc, char** argv) {
    ros::init(argc, argv, "segmentation_node");
    ros::NodeHandle nh;
    // ros::Subscriber sub = nh.subscribe("/camera/depth/points", 1, pcCallback);
    // pub = nh.advertise<sensor_msgs::PointCloud2>("/processed/pointcloud", 1);
    ros::Rate loop_rate(20);
    Eigen::Vector3f nominal_road_normal(0.0, 0.0, 1.0);
    ImageConverter ic;
    ros::spin();
    // while (ros::ok()) {
    //     ros::spinOnce();
    //     pub.publish(cloud_publish);
    //     loop_rate.sleep();
    // }
}