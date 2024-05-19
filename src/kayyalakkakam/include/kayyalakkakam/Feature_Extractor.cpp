#include "Feature_Extractor.h"

FeatureExtractor::FeatureExtractor(ros::NodeHandle& nh) : nh_(nh) {
    scan_sub_ = nh_.subscribe("/scan", 10, &FeatureExtractor::scanCallback, this);
    features_marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("visualization_feature_marker", 10);
}

void FeatureExtractor::scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan) {
    sensor_msgs::PointCloud2 cloud2;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    
    // projector_.projectLaser(*scan, cloud2);
    projector_.transformLaserScanToPointCloud("map", *scan, cloud2, transformer);
    pcl::fromROSMsg(cloud2, *cloud);
    
    detectFeatures(cloud);
    saveFeaturesToFile("/home/cocokayya18/Probabilistic-Robotics-Lab/src/kayyalakkakam/src/features/features.csv");
    publishMarkers();
}

void FeatureExtractor::detectFeatures(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud) {
    features_.clear();
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(cloud);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance(0.2); // 20cm between points to be considered in the same cluster
    ec.setMinClusterSize(10);
    ec.setMaxClusterSize(250);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud);
    ec.extract(cluster_indices);

    for (auto it = cluster_indices.begin(); it != cluster_indices.end(); ++it) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZ>);
        for (const auto& idx : it->indices)
            cloud_cluster->push_back((*cloud)[idx]);

        Eigen::Vector4f centroid;
        pcl::compute3DCentroid(*cloud_cluster, centroid);
        features_.push_back(std::make_pair(centroid[0], centroid[1]));
    }
}

void FeatureExtractor::saveFeaturesToFile(const std::string& filename) {
    std::ofstream file;
    file.open(filename);
    for (const auto& feature : features_) {
        file << feature.first << "," << feature.second << "\n";
    }
    
    file.close();
}

void FeatureExtractor::publishMarkers() {
    visualization_msgs::MarkerArray markers;
    for (size_t i = 0; i < features_.size(); ++i) {
        visualization_msgs::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = ros::Time::now();
        marker.ns = "features";
        marker.id = i;
        marker.type = visualization_msgs::Marker::CYLINDER;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = features_[i].first;
        marker.pose.position.y = features_[i].second;
        marker.pose.position.z = 0.5;
        marker.scale.x = 0.2;
        marker.scale.y = 0.2;
        marker.scale.z = 1.0;
        marker.color.a = 1.0;
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;
        markers.markers.push_back(marker);
    }

    features_marker_pub_.publish(markers);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "Feature Extractor Node");
    ROS_INFO_STREAM("Feature Extractor Node started");
    ros::NodeHandle nh("~");
    FeatureExtractor featureExtractor(nh);

    ros::Rate loop_rate(10);

    while (ros::ok()) {
        ros::spinOnce();
        loop_rate.sleep();
    }

    ROS_INFO_STREAM("CLOSING EVERYTHING");
    return 0;
}
