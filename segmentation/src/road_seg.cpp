#include <segmentation/road_seg.hpp>
#define FRAC_SAMPLE 0.001          // Fraction of points to be sampled for use in computeAverageZ and computeDeltaZ
#define MIN_POINTS_ROAD 35000      // Minimum number of points in a road segment. If less, then a penalty will be inposed.
#define DELTAZ_PARAM 1.35          // Minimum ratio of first and second lowest deltaZ values
#define ROLLING_AVERAGE_PARAM 0.1  // Gamma for moving average
#define SMALL_SIZE_PENALTY 30.0    // Penalty for small road candidates
#define PERPENDICULAR_LIMIT 170.0
#define AREA_THRESHOLD 0.85
#define NEXT_WAYPOINT_DIS 0.35
namespace road_detector {

RoadDetector::RoadDetector()
    : it_(nh_) {
    // Subscribe to input video feed and publish output video feed
    image_sub_ = it_.subscribe("/depth_camera/rgb/image_raw", 1, &RoadDetector::imageCb, this);
    image_pub_ = it_.advertise("/image_converter/output_video", 1);
    cloud_sub_ = nh_.subscribe("/depth_camera/depth/points", 1, &RoadDetector::pcCallback, this);
    cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/processed/pointcloud", 1);
    waypoint_pub_ = nh_.advertise<segmentation::drone_way>("/drone_way", 1);
    cv::namedWindow(OPENCV_WINDOW);

    // Camera parameters
    fx = 554.254691191187;
    cx = 320.5;
    fy = 554.254691191187;
    cy = 240.5;

    // Rolling average Z for robust road decisions
    prevZ = 0.0;

    // Planar segmentation object
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(0.1);
}

std::vector<int> RoadDetector::randomPick(int N, int k) {
    // Picks k distinct random numbers from 0 to N-1
    std::mt19937 gen(rd());  // Standard mersenne_twister_engine seeded with rd()

    for (int r = N - k; r < N; ++r) {
        int v = std::uniform_int_distribution<>(1, r)(gen);
        if (!elems.insert(v).second) {
            elems.insert(r);
        }
    }
    std::vector<int> result(elems.begin(), elems.end());
    std::shuffle(result.begin(), result.end(), gen);
    elems.clear();
    return result;  // vector of random numbers
}

float RoadDetector::computeAverageZ(pcl::PointCloud<PointT>::Ptr& cloud, pcl::PointIndices::Ptr inliers, float fraction, int i) {
    int total = inliers->indices.size();
    int k = (int) (total * fraction);  // number of points to sample
    float Z = 0;
    std::vector<int> random_list = randomPick(total, k);
    for (int j = 0; j < k; ++j) {
        Z += cloud->points[inliers->indices[random_list[j]]].z;  // add every point's Z-coordinate
    }
    Z = Z / k;  // average
    averageZ[i] = Z;
    return Z;
}

float RoadDetector::computeDeltaZ(pcl::PointCloud<PointT>::Ptr& cloud, pcl::PointIndices::Ptr inliers, float fraction) {
    int total = inliers->indices.size();
    int k = (int) (total * fraction);
    float deltaZ = 0;

    std::vector<int> random_list = randomPick(total, k);
    for (int i = 0; i < k; ++i) {
        for (int j = i + 1; j < k; ++j) {
            // slope computation below
            // vertical_rise/ horizontal_distance
            float delZ = abs(cloud->points[random_list[i]].z - cloud->points[random_list[j]].z);
            float delX = cloud->points[random_list[i]].x - cloud->points[random_list[j]].x;
            float delY = cloud->points[random_list[i]].y - cloud->points[random_list[j]].y;
            float del = delZ / sqrt(delX * delX + delY * delY);
            if (!isnan(del))  // NaN check
                deltaZ += del;
        }
    }
    int pairs = (k * (k - 1)) / 2;  // kC2
    deltaZ = deltaZ / pairs;        // average
    std::cout << "deltaZ = " << deltaZ << std::endl;
    return deltaZ;
}

void RoadDetector::imageCb(const sensor_msgs::ImageConstPtr& msg) {
    cv_bridge::CvImagePtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    img_[0] = cv_ptr->image.clone();
    img_[1] = cv_ptr->image.clone();
    img_[2] = cv_ptr->image.clone();

    // std::cout<<(cv_ptr->image).rows<<" "<<(cv_ptr->image).cols<<std::endl;

    std::cout << "[ImageCb] Publishing image after drawing circle" << std::endl;
    image_pub_.publish(cv_ptr->toImageMsg());
}

float RoadDetector::computeAlignment(pcl::PointCloud<PointT>::Ptr& cloud, pcl::PointIndices::Ptr inliers) {
    // Computes normal alignment of the road plane. Returns dot of average normal with (0,0,-1)

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr label_image(new pcl::PointCloud<pcl::PointXYZRGB>);
    for (const auto& idx : inliers->indices) {
        PointT n = cloud->points[idx];
        n.r = 255;
        label_image->points.push_back(n);
    }
    pcl::NormalEstimation<PointT, pcl::Normal> ne;
    ne.setInputCloud(label_image);
    pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>());
    ne.setSearchMethod(tree);
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
    ne.setKSearch(50);
    ne.compute(*cloud_normals);

    pcl::Normal normal_at_point;
    pcl::PointXYZRGB point;
    float alignment = 0.0;
    int count = 0;

    for (const auto& n : cloud_normals->points) {
        std::cout << n.normal_x << " " << n.normal_y << " " << n.normal_z << std::endl;
        if (!isnan(n.normal_x) && (!isnan(n.normal_y)) && (!isnan(n.normal_z))) {
            alignment -= n.normal_z;
            ++count;
        }
    }
    return alignment / count;
}

void RoadDetector::makeImageBlack(int i) {
    for (int y = 0; y < img_[i].rows; y++) {
        for (int x = 0; x < img_[i].cols; x++) {
            img_[i].at<cv::Vec3b>(y, x)[0] = 0;
            img_[i].at<cv::Vec3b>(y, x)[1] = 0;
            img_[i].at<cv::Vec3b>(y, x)[2] = 0;
        }
    }
}

void RoadDetector::imageFromPointcloud(pcl::PointCloud<PointT>::Ptr cloud,
    pcl::PointIndices::Ptr inliers,
    pcl::PointCloud<PointT>::Ptr& label_image,
    int colour,
    int i) {
    for (const auto& idx : inliers->indices) {
        PointT n = cloud->points[idx];
        switch (colour) {
            case 0: {
                n.r = 255;
                break;
            }
            case 1: {
                n.g = 255;
                break;
            }
            case 2: {
                n.b = 255;
                break;
            }
        }
        label_image->points.push_back(n);
        if (n.z != 0) {
            float m1 = n.x / n.z;
            float n1 = n.y / n.z;

            int x = int(m1 * fx + cx);
            int y = int(n1 * fy + cy);
            if (x < img_[i].cols && y < img_[i].rows) {
                img_[i].at<cv::Vec3b>(y, x)[0] = colour == 2 ? 255 : 0;
                img_[i].at<cv::Vec3b>(y, x)[1] = colour == 1 ? 255 : 0;
                img_[i].at<cv::Vec3b>(y, x)[2] = colour == 0 ? 255 : 0;
            }
        }
    }
}

void RoadDetector::extractFromInliers(pcl::PointCloud<PointT>::Ptr in, pcl::PointCloud<PointT>::Ptr& out, pcl::PointIndices::Ptr inliers) {
    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(in);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*out);
}

void RoadDetector::drawAxis(cv::Mat& img, cv::Point p, cv::Point q, cv::Scalar colour, const float scale = 0.2) {
    double angle = atan2((double) p.y - q.y, (double) p.x - q.x);  // angle in radians
    double hypotenuse = sqrt((double) (p.y - q.y) * (p.y - q.y) + (p.x - q.x) * (p.x - q.x));
    // Here we lengthen the arrow by a factor of scale
    q.x = (int) (p.x - scale * hypotenuse * cos(angle));
    q.y = (int) (p.y - scale * hypotenuse * sin(angle));
    cv::line(img, p, q, colour, 1, cv::LINE_AA);
    // create the arrow hooks
    p.x = (int) (q.x + 9 * cos(angle + CV_PI / 4));
    p.y = (int) (q.y + 9 * sin(angle + CV_PI / 4));
    cv::line(img, p, q, colour, 1, cv::LINE_AA);
    p.x = (int) (q.x + 9 * cos(angle - CV_PI / 4));
    p.y = (int) (q.y + 9 * sin(angle - CV_PI / 4));
    cv::line(img, p, q, colour, 1, cv::LINE_AA);
}

double RoadDetector::getOrientation(const std::vector<cv::Point>& pts, cv::Mat& img) {
    // Construct a buffer used by the pca analysis
    int sz = static_cast<int>(pts.size());
    cv::Mat data_pts = cv::Mat(sz, 2, CV_64F);
    for (int i = 0; i < data_pts.rows; i++) {
        data_pts.at<double>(i, 0) = pts[i].x;
        data_pts.at<double>(i, 1) = pts[i].y;
    }
    // Perform PCA analysis
    cv::PCA pca_analysis(data_pts, cv::Mat(), cv::PCA::DATA_AS_ROW);
    // Store the center of the object
    cv::Point cntr = cv::Point(static_cast<int>(pca_analysis.mean.at<double>(0, 0)), static_cast<int>(pca_analysis.mean.at<double>(0, 1)));
    // center.x center.y

    // Store the eigenvalues and eigenvectors
    std::vector<cv::Point2d> eigen_vecs(2);
    std::vector<double> eigen_val(2);
    for (int i = 0; i < 2; i++) {
        eigen_vecs[i] = cv::Point2d(pca_analysis.eigenvectors.at<double>(i, 0), pca_analysis.eigenvectors.at<double>(i, 1));
        eigen_val[i] = pca_analysis.eigenvalues.at<double>(i);
    }
    // Draw the principal components
    cv::circle(img, cntr, 3, cv::Scalar(255, 0, 255), 2);
    cv::Point p1 = cntr + 0.02 * cv::Point(static_cast<int>(eigen_vecs[0].x * eigen_val[0]), static_cast<int>(eigen_vecs[0].y * eigen_val[0]));
    cv::Point p2 = cntr - 0.02 * cv::Point(static_cast<int>(eigen_vecs[1].x * eigen_val[1]), static_cast<int>(eigen_vecs[1].y * eigen_val[1]));
    // std::cout<<p1.x<<" "<<p1.y<<"\n";
    cv::Point dir;
    dir.x = p1.x - cntr.x;
    dir.y = p1.y - cntr.y;
    if ((-(dir.y)) < 0) {
        dir.x = (-dir.x);
        dir.y = (-dir.y);
        p1.x = cntr.x + 10 * dir.x;
        p1.y = cntr.y + 10 * dir.y;
    }
    cv::Point image_center;
    image_center.x = img.cols / 2;
    image_center.y = img.rows / 2;
    next_waypoint_.corrective = 0;
    if (get_perpendicular_distance(p1, image_center, cntr) > PERPENDICULAR_LIMIT) {
        float t = dir.x;
        dir.x = dir.y;
        dir.y = -t;

        if ((image_center.x - cntr.x) * (dir.x) + (image_center.y - cntr.y) * (dir.y) > 0) {
            dir.x = (-dir.x);
            dir.y = (-dir.y);
        }
        next_waypoint_.corrective = 1;
    }
    waypoint.x = cntr.x + (NEXT_WAYPOINT_DIS * dir.x);
    waypoint.y = cntr.y + (NEXT_WAYPOINT_DIS * dir.y);
    std::cout << "waypoint: " << waypoint.x << " " << waypoint.y << "\n";
    cv::circle(img, waypoint, 3, cv::Scalar(255, 255, 255), 2);
    drawAxis(img, cntr, p1, cv::Scalar(0, 0, 0), 1);
    // drawAxis(img, cntr, p2, cv::Scalar(255, 255,0 ), 5);
    double angle = atan2(eigen_vecs[0].y, eigen_vecs[0].x);  // orientation in radians

    // Construct the waypoint
    next_waypoint_.x = (*cloud_).at(waypoint.x, waypoint.y).x;
    next_waypoint_.y = (*cloud_).at(waypoint.x, waypoint.y).y;
    next_waypoint_.z = (*cloud_).at(waypoint.x, waypoint.y).z;
    next_waypoint_.theta = angle;

    // drawAxis(img, cntr, p2, cv::Scalar(255, 255,0 ), 5);
    // double angle = atan2(eigen_vecs[0].y, eigen_vecs[0].x);  // orientation in radians
    return angle - 1.57;  // change in yaw required
}
float RoadDetector::get_perpendicular_distance(const cv::Point p, const cv::Point q, const cv::Point r) {
    float area = fabs((p.x - r.x) * (q.y - r.y) - (q.x - r.x) * (p.y - r.y));
    float dist = area / cv::norm(p - r);
    std::cout << "dist: " << dist << "\n";
    return dist;
    // q is the image center
}
void RoadDetector::meanPath() {
    cv::Mat gray;
    cv::cvtColor(img_[min_alignment_idx], gray, CV_BGR2GRAY);

    cv::Mat blur;
    cv::GaussianBlur(gray, blur, cv::Size(15, 15), 0);

    cv::Mat thresh;
    cv::threshold(blur, thresh, 70, 255, CV_THRESH_BINARY | CV_THRESH_OTSU);

    // cv::GaussianBlur(thresh, thresh, cv::Size(9, 9), 0);

    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(15, 15));

    cv::morphologyEx(thresh, thresh, cv::MORPH_OPEN, kernel);

    std::vector<cv::Point> Image_Pts;

    cv::Point center;

    for (int x = 0; x < img_[min_alignment_idx].cols; x++) {
        for (int y = 0; y < img_[min_alignment_idx].rows; y++) {
            if (thresh.at<uchar>(y, x) == 255) {
                cv::Point p;
                p.x = x;
                p.y = y;
                center.x += x;
                center.y += y;
                Image_Pts.push_back(p);
            }
        }
    }

    if(((float)Image_Pts.size()/(img_[min_alignment_idx].cols*img_[min_alignment_idx].rows))>AREA_THRESHOLD){
        std::cout<<"here\n";
        cv::Point img_center;
        img_center.x=img_[min_alignment_idx].cols/2;
        img_center.y=img_[min_alignment_idx].rows/2;
        cv::Point straight;
        straight.x=img_center.x;
        straight.y= img_center.y - 100;
        waypoint.x=0;
        waypoint.y=-1;
        cv::circle(img_[min_alignment_idx], waypoint, 3, cv::Scalar(255, 0, 255), 2);
        drawAxis(img_[min_alignment_idx],img_center,straight,cv::Scalar(0, 0, 0), 1);
        next_waypoint_.x = (*cloud_).at(straight.x, straight.y).x;
        next_waypoint_.y = (*cloud_).at(straight.x, straight.y).y;
        next_waypoint_.z = (*cloud_).at(straight.x, straight.y).z;
        next_waypoint_.theta = 1.57 - 1.57;
        next_waypoint_.corrective = 0;
    }
    else{
        getOrientation(Image_Pts,img_[min_alignment_idx]);
    }

    try {
        cv::imshow("Final", img_[min_alignment_idx]);
        cv::waitKey(3);
    } catch (...) {
        std::cout << "imshow error\n";
    }
}

void RoadDetector::pcCallback(const sensor_msgs::PointCloud2::ConstPtr& input) {
    pcl_conversions::toPCL(*input, pcl_pc2);
    pcl::fromPCLPointCloud2(pcl_pc2, *cloud_);

    min_alignment_idx = 0;

    seg.setInputCloud(cloud_);
    seg.segment(*inliers, *coefficients);

    int i = 0;
    for (int i = 0; i < 2; i++) {
        makeImageBlack(i);
    }

    i = 0;
    imageFromPointcloud(cloud_, inliers, label_image, 0, i);
    i++;
    alignment.push_back({computeDeltaZ(cloud_, inliers, FRAC_SAMPLE), 0});
    // Store deltaZ value of plane as a pair. Sorting will be done based on the first value
    if (inliers->indices.size() < MIN_POINTS_ROAD)
        alignment[0].first += SMALL_SIZE_PENALTY;  // Impose penalty if the pointcloud is too small

    computeAverageZ(cloud_, inliers, FRAC_SAMPLE, 0);

    extractFromInliers(cloud_, next, inliers);
    seg.setInputCloud(next);
    seg.segment(*inliers, *coefficients);

    imageFromPointcloud(next, inliers, label_image, 1, i);

    alignment.push_back({computeDeltaZ(next, inliers, FRAC_SAMPLE), 1});
    if (inliers->indices.size() < MIN_POINTS_ROAD)
        alignment[1].first += SMALL_SIZE_PENALTY;

    computeAverageZ(next, inliers, FRAC_SAMPLE, 1);
    // computeAverageZ(next, inliers, FRAC_SAMPLE, 1);
    i++;
    extractFromInliers(next, next2, inliers);
    seg.setInputCloud(next2);
    seg.segment(*inliers, *coefficients);
    imageFromPointcloud(next2, inliers, label_image, 2, i);
    alignment.push_back({computeDeltaZ(next2, inliers, FRAC_SAMPLE), 2});

    if (inliers->indices.size() < MIN_POINTS_ROAD)
        alignment[2].first += SMALL_SIZE_PENALTY;
    computeAverageZ(next2, inliers, FRAC_SAMPLE, 2);
    i++;

    // sort the alignment vector based on the first value of the pair
    sort(alignment.begin(), alignment.end());
    std::vector<std::pair<float, int>> final_alignment;
    final_alignment.push_back(alignment[0]);
    if (alignment[1].first < (alignment[0].first) * DELTAZ_PARAM)
        final_alignment.push_back(alignment[1]);
    if (alignment[2].first < (alignment[0].first) * DELTAZ_PARAM)
        final_alignment.push_back(alignment[2]);
    sort(final_alignment.begin(), final_alignment.end());
    for (int i = 0; i < final_alignment.size(); i++) {
        final_alignment[i].first = fabs(averageZ[final_alignment[i].second] - prevZ);
    }
    sort(final_alignment.begin(), final_alignment.end());
    min_alignment_idx = final_alignment[0].second;

    // We use a moving mean to update the prevZ value. This is to prevent the prevZ value from being
    // thrown off by a single bad detection. It cheaply averages the last few detections to get a more
    // stable and reliable averageZ values.
    prevZ = averageZ[min_alignment_idx] * ROLLING_AVERAGE_PARAM + prevZ * (1 - ROLLING_AVERAGE_PARAM);
    sensor_msgs::PointCloud2 cloud_publish;
    pcl::toROSMsg(*label_image, cloud_publish);
    cloud_publish.header = input->header;
    cloud_pub_.publish(cloud_publish);
    try {
        cv::imshow(OPENCV_WINDOW, img_[0]);
        cv::imshow("Second", img_[1]);
        cv::imshow("Third", img_[2]);
        cv::waitKey(3);
        meanPath();
    } catch (...) {
        std::cout << "imshow error\n";
    }
    alignment.clear();
    final_alignment.clear();

    waypoint_pub_.publish(next_waypoint_);
}
}  // namespace road_detector