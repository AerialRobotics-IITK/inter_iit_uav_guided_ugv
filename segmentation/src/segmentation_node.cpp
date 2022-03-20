#include <ros/console.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
// PCL specific includes
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
// // /camera/depth/points
typedef pcl::PointXYZRGB PointT;
ros::Publisher pub;
pcl::IntegralImageNormalEstimation<PointT, pcl::Normal> ne;
pcl::GroundPlaneComparator<PointT, pcl::Normal>::Ptr road_comparator(new pcl::GroundPlaneComparator<PointT, pcl::Normal>);
pcl::OrganizedConnectedComponentSegmentation<PointT, pcl::Label> road_segmentation(road_comparator);
void pcCallback(const sensor_msgs::PointCloud2::ConstPtr& input) {
    // Create a container for the data. --------------------------------------------------------------------------------
    pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(*input, pcl_pc2);
    pcl::fromPCLPointCloud2(pcl_pc2, *cloud);

    // Compute the normals-- -----------------------------------------------------------------------------------------------
    pcl::PointCloud<pcl::Normal>::Ptr normal_cloud(new pcl::PointCloud<pcl::Normal>);
    ne.setInputCloud(cloud);
    ne.compute(*normal_cloud);

    // Set up the groundplane comparator
    road_comparator->setInputCloud(cloud);
    road_comparator->setInputNormals(normal_cloud);

    // Run segmentation
    pcl::PointCloud<pcl::Label> labels;
    std::vector<pcl::PointIndices> region_indices;
    road_segmentation.setInputCloud(cloud);
    road_segmentation.segment(labels, region_indices);

    // Draw the segmentation result
    pcl::PointCloud<pcl::PointXYZ>::Ptr ground_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr ground_image(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr label_image(new pcl::PointCloud<pcl::PointXYZRGB>);
    *ground_image = *cloud;
    *label_image = *cloud;

    Eigen::Vector4f clust_centroid = Eigen::Vector4f::Zero();
    Eigen::Vector4f vp = Eigen::Vector4f::Zero();
    Eigen::Matrix3f clust_cov;
    pcl::ModelCoefficients model;
    model.values.resize(4);

    std::vector<pcl::ModelCoefficients> model_coefficients;
    std::vector<Eigen::Vector4f, Eigen::aligned_allocator<Eigen::Vector4f> > centroids;
    std::vector<Eigen::Matrix3f, Eigen::aligned_allocator<Eigen::Matrix3f> > covariances;
    std::vector<pcl::PointIndices> inlier_indices;

    for (const auto& region_index : region_indices) {
        if (region_index.indices.size() > 1000) {
            for (size_t j = 0; j < region_index.indices.size(); j++) {
                pcl::PointXYZ ground_pt(
                    cloud->points[region_index.indices[j]].x, cloud->points[region_index.indices[j]].y, cloud->points[region_index.indices[j]].z);
                ground_cloud->points.push_back(ground_pt);
                ground_image->points[region_index.indices[j]].g = static_cast<pcl::uint8_t>((cloud->points[region_index.indices[j]].g + 255) / 2);
                label_image->points[region_index.indices[j]].r = 0;
                label_image->points[region_index.indices[j]].g = 255;
                label_image->points[region_index.indices[j]].b = 0;
            }

            // Compute plane info
            pcl::computeMeanAndCovarianceMatrix(*cloud, region_index.indices, clust_cov, clust_centroid);
            Eigen::Vector4f plane_params;

            EIGEN_ALIGN16 Eigen::Vector3f::Scalar eigen_value;
            EIGEN_ALIGN16 Eigen::Vector3f eigen_vector;
            pcl::eigen33(clust_cov, eigen_value, eigen_vector);
            plane_params[0] = eigen_vector[0];
            plane_params[1] = eigen_vector[1];
            plane_params[2] = eigen_vector[2];
            plane_params[3] = 0;
            plane_params[3] = -1 * plane_params.dot(clust_centroid);

            vp -= clust_centroid;
            float cos_theta = vp.dot(plane_params);
            if (cos_theta < 0) {
                plane_params *= -1;
                plane_params[3] = 0;
                plane_params[3] = -1 * plane_params.dot(clust_centroid);
            }

            model.values[0] = plane_params[0];
            model.values[1] = plane_params[1];
            model.values[2] = plane_params[2];
            model.values[3] = plane_params[3];
            model_coefficients.push_back(model);
            inlier_indices.push_back(region_index);
            centroids.push_back(clust_centroid);
            covariances.push_back(clust_cov);
        }
    }

    // Refinement
    std::vector<bool> grow_labels;
    std::vector<int> label_to_model;
    grow_labels.resize(region_indices.size(), false);
    label_to_model.resize(region_indices.size(), 0);

    for (size_t i = 0; i < model_coefficients.size(); i++) {
        int model_label = (labels)[inlier_indices[i].indices[0]].label;
        label_to_model[model_label] = static_cast<int>(i);
        grow_labels[model_label] = true;
    }

    boost::shared_ptr<pcl::PointCloud<pcl::Label> > labels_ptr(new pcl::PointCloud<pcl::Label>());
    *labels_ptr = labels;
    pcl::OrganizedMultiPlaneSegmentation<PointT, pcl::Normal, pcl::Label> mps;
    pcl::PlaneRefinementComparator<PointT, pcl::Normal, pcl::Label>::Ptr refinement_compare(
        new pcl::PlaneRefinementComparator<PointT, pcl::Normal, pcl::Label>());
    refinement_compare->setInputCloud(cloud);
    refinement_compare->setDistanceThreshold(0.15f);
    refinement_compare->setLabels(labels_ptr);
    refinement_compare->setModelCoefficients(model_coefficients);
    refinement_compare->setRefineLabels(grow_labels);
    refinement_compare->setLabelToModel(label_to_model);
    mps.setRefinementComparator(refinement_compare);
    mps.setMinInliers(500);
    mps.setAngularThreshold(pcl::deg2rad(5.0));
    mps.setDistanceThreshold(0.01);
    mps.setInputCloud(cloud);
    mps.setInputNormals(normal_cloud);
    mps.refine(model_coefficients, inlier_indices, centroids, covariances, labels_ptr, region_indices);

    // Note the regions that have been extended
    pcl::PointCloud<PointT> extended_ground_cloud;
    for (const auto& region_index : region_indices) {
        if (region_index.indices.size() > 1000) {
            for (size_t j = 0; j < region_index.indices.size(); j++) {
                // Check to see if it has already been labeled
                if (ground_image->points[region_index.indices[j]].g == ground_image->points[region_index.indices[j]].b) {
                    pcl::PointXYZ ground_pt(
                        cloud->points[region_index.indices[j]].x, cloud->points[region_index.indices[j]].y, cloud->points[region_index.indices[j]].z);
                    ground_cloud->points.push_back(ground_pt);
                    ground_image->points[region_index.indices[j]].r = static_cast<pcl::uint8_t>((cloud->points[region_index.indices[j]].r + 255) / 2);
                    ground_image->points[region_index.indices[j]].g = static_cast<pcl::uint8_t>((cloud->points[region_index.indices[j]].g + 255) / 2);
                    label_image->points[region_index.indices[j]].r = 128;
                    label_image->points[region_index.indices[j]].g = 128;
                    label_image->points[region_index.indices[j]].b = 0;
                }
            }
        }
    };
    // pcl::PointCloud<pcl::PointXYZRGB>::Ptr label_image(new pcl::PointCloud<pcl::PointXYZRGB>);
    // for (PointT point : cloud->points) {
    //     PointT n = point;
    //     n.r = 255;
    //     label_image->points.push_back(n);
    // }
    sensor_msgs::PointCloud2 cloud_publish;
    pcl::toROSMsg(*label_image, cloud_publish);
    cloud_publish.header = input->header;

    pub.publish(cloud_publish);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "segmentation_node");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("/camera/depth/points", 1, pcCallback);
    pub = nh.advertise<sensor_msgs::PointCloud2>("/processed/pointcloud", 1);
    ros::Rate loop_rate(20);
    Eigen::Vector3f nominal_road_normal(0.0, 0.0, 1.0);
    // Adjust for camera tilt:
    Eigen::Vector3f tilt_road_normal = Eigen::AngleAxisf(pcl::deg2rad(14.5f), Eigen::Vector3f::UnitX()) * nominal_road_normal;
    road_comparator->setExpectedGroundNormal(tilt_road_normal);
    road_comparator->setGroundAngularThreshold(pcl::deg2rad(10.0f));
    road_comparator->setAngularThreshold(pcl::deg2rad(6.0f));

    // -----------------------------------------------------------------------------------------------------------------
    ne.setNormalEstimationMethod(ne.COVARIANCE_MATRIX);
    ne.setMaxDepthChangeFactor(0.10f);
    ne.setNormalSmoothingSize(40.0f);  // 20.0f
    ros::spin();
    // while (ros::ok()) {
    //     ros::spinOnce();
    //     pub.publish(cloud_publish);
    //     loop_rate.sleep();
    // }
}