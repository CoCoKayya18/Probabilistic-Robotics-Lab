#include "Feature_Extractor.h"

FeatureExtractor::FeatureExtractor(ros::NodeHandle& nh) : nh_(nh), robot(nh), tfBuffer(ros::Duration(10)), tfListener(tfBuffer) {
    // scan_sub_ = nh_.subscribe("/scan", 10, &FeatureExtractor::scanCallback, this);
    features_marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("visualization_feature_marker", 10);
}

void FeatureExtractor::update() {

    scan = robot.getLaserscan();

    sensor_msgs::PointCloud2 cloud2;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

    if (!transformer.waitForTransform("map", "base_scan", scan.header.stamp, ros::Duration(0.5))) {
    ROS_WARN("Waiting for the transform timed-out.");
    return;
    }
    

    try {
    projector_.transformLaserScanToPointCloud("map", scan, cloud2, transformer);
    } 
    catch (tf::TransformException &ex) {
        ROS_WARN("TF exception:\n%s", ex.what());
        return;  // Optionally delay and retry
    }
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
    ec.setClusterTolerance(0.75); 
    ec.setMinClusterSize(20);
    ec.setMaxClusterSize(10000);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud);
    ec.extract(cluster_indices);

    for (auto it = cluster_indices.begin(); it != cluster_indices.end(); ++it) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZ>);
        for (const auto& idx : it->indices)
            cloud_cluster->push_back((*cloud)[idx]);

        Eigen::Vector4f centroid;
        pcl::compute3DCentroid(*cloud_cluster, centroid);
        // features_.push_back(std::make_pair(centroid[0], centroid[1]));

        // Filter to add features based on distance from a hypothetical robot center
        updateRobotPosition();
        float distance_from_robot = sqrt(pow(centroid[0] - robot_x, 2) + pow(centroid[1] - robot_y, 2));
        if (distance_from_robot > min_distance_to_consider) {  // Set a minimum distance
            // features_.push_back(std::make_pair(centroid[0], centroid[1]));
            updateFeatureMap(std::make_pair(centroid[0], centroid[1]));
        }

    }
}

void FeatureExtractor::updateRobotPosition()
{
    geometry_msgs::TransformStamped transformStamped;
    try {
        // Ensure to get the transform at the appropriate time
        transformStamped = tfBuffer.lookupTransform("map", "base_footprint", scan.header.stamp, ros::Duration(1.0));
        
        // Transform to map frame and update robots position
        robot_x = transformStamped.transform.translation.x;
        robot_y = transformStamped.transform.translation.y;
        // ROS_INFO_STREAM("Robots position in map frame: " << robot_x << ", " << robot_y);
    } catch (tf2::TransformException &ex) {
        ROS_ERROR("%s", ex.what());
        return;
    }
}

void FeatureExtractor::saveFeaturesToFile(const std::string& filename) {
    std::ofstream file;
    file.open(filename, std::ios::out | std::ios::app); 
    for (const auto& feature : feature_map_) {
        file << std::fixed << std::setprecision(20) << feature.first.first << "," << feature.first.second << "," << feature.second << "\n";
    }
    file.close();
}

void FeatureExtractor::publishMarkers() {
        visualization_msgs::MarkerArray markers;
    int id = 0;
    for (const auto& feature : feature_map_) {
        visualization_msgs::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = ros::Time::now();
        marker.ns = "features";
        marker.id = id++;  
        marker.type = visualization_msgs::Marker::SPHERE;  
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = feature.first.first;
        marker.pose.position.y = feature.first.second;
        marker.pose.position.z = 0.1;  
        marker.scale.x = 0.2;  
        marker.scale.y = 0.2;
        marker.scale.z = 0.2;
        marker.color.a = 1.0;  
        marker.color.r = 1.0;  
        marker.color.g = 0.0;
        marker.color.b = 0.0;
        markers.markers.push_back(marker);
    }
    features_marker_pub_.publish(markers);
}

void FeatureExtractor::updateFeatureMap(const std::pair<float, float>& feature)
{
    double tolerance = 0.001; // Adjust the tolerance as needed
    bool feature_exists = false;
    for (const auto& existing_feature : feature_map_) {
        double diff_x = std::abs(existing_feature.first.first - feature.first);
        double diff_y = std::abs(existing_feature.first.second - feature.second);
        // ROS_INFO_STREAM("Comparing with existing feature: (" << existing_feature.first.first << ", " << existing_feature.first.second << ")");
        // ROS_INFO_STREAM("Differences: diff_x = " << diff_x << ", diff_y = " << diff_y);
        if (diff_x < tolerance && diff_y < tolerance) {
            // ROS_INFO_STREAM("Feature already exists");
            feature_exists = true;
            break;
        }
    }

    if (!feature_exists) {
        ROS_INFO_STREAM("NEW FEATURE FOUND");
        feature_map_[feature] = feature_id_++;
    }
}

void FeatureExtractor::removeDuplicateFeatures() {
    std::unordered_map<int, std::pair<float, float>> unique_features_by_index;
    std::unordered_map<std::pair<float, float>, int, PointHash, PointEqual> cleaned_feature_map;
    std::unordered_set<int> seen_indices;
    
    // Iterate through the feature_map_ and keep only unique indices
    for (const auto& feature : feature_map_) {
        int feature_id = feature.second;
        std::pair<float, float> feature_coords = feature.first;
        
        ROS_INFO_STREAM("Feature ID: " << feature_id << ", Coordinates: (" << feature_coords.first << ", " << feature_coords.second << ")");
        
        if (seen_indices.find(feature_id) == seen_indices.end()) {
            unique_features_by_index[feature_id] = feature_coords;
            seen_indices.insert(feature_id);
        }
    }
    
    // Rebuild the cleaned_feature_map
    for (const auto& feature : unique_features_by_index) {
        cleaned_feature_map[feature.second] = feature.first;
    }
    
    // Replace the original feature_map_ with the cleaned map
    feature_map_ = std::move(cleaned_feature_map);
    
    ROS_INFO_STREAM("Feature map cleaned. Remaining unique features:");
    for (const auto& feature : feature_map_) {
        ROS_INFO_STREAM("Feature ID: " << feature.second << ", Coordinates: (" << feature.first.first << ", " << feature.first.second << ")");
    }
}


int main(int argc, char** argv) {
    ros::init(argc, argv, "Feature Extractor Node");
    ROS_INFO_STREAM("Feature Extractor Node started");
    ros::NodeHandle nh("~");
    FeatureExtractor featureExtractor(nh);

    ros::Rate loop_rate(10);

    while (ros::ok()) {
        featureExtractor.update();
        featureExtractor.removeDuplicateFeatures();
        ros::spinOnce();
        loop_rate.sleep();
    }

    ROS_INFO_STREAM("CLOSING EVERYTHING");
    return 0;
}
