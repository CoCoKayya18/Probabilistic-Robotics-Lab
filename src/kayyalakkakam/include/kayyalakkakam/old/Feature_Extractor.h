#ifndef FEATURE_EXTRACTOR_H
#define FEATURE_EXTRACTOR_H

#include <vector>
#include <string>
#include <cmath>
#include <fstream>
#include "robot_class.h"
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <tf/transform_listener.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <laser_geometry/laser_geometry.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/centroid.h>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <functional> 
#include <csignal>  
#include <pthread.h>

struct PointHash {
    std::size_t operator()(const std::pair<float, float>& p) const {
        auto hash1 = std::hash<float>{}(p.first);
        auto hash2 = std::hash<float>{}(p.second);
        return hash1 ^ hash2; // Combine hashes
    }
};

struct PointEqual {
    bool operator()(const std::pair<float, float>& p1, const std::pair<float, float>& p2) const {
        double tolerance = 0.001; // Adjust tolerance if needed
        return std::fabs(p1.first - p2.first) < tolerance && std::fabs(p1.second - p2.second) < tolerance;
    }
};

class FeatureExtractor 
{
    public:
        FeatureExtractor(ros::NodeHandle& nh);
        void update();
        void detectFeatures(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);
        void saveFeaturesToFile(const std::string& filename);
        void publishMarkers();
        void updateFeatureMap(const std::pair<float, float>& feature);
        void updateRobotPosition();
        void removeDuplicateFeatures();

    private:
        ros::NodeHandle nh_;
        // ros::Subscriber scan_sub_;
        sensor_msgs::LaserScan scan;
        nav_msgs::Odometry odom;
        Ros_Subscriber_Publisher_Class robot;
        ros::Publisher features_marker_pub_;
        std::vector<std::pair<double, double>> features_;
        laser_geometry::LaserProjection projector_; // To convert LaserScan to PointCloud
        
        tf::TransformListener transformer;

        tf2_ros::Buffer tfBuffer;
        tf2_ros::TransformListener tfListener;

        // Map vector and feature id
        std::unordered_map<std::pair<float, float>, int, PointHash, PointEqual> feature_map_;
        int feature_id_ = 0; 

        double min_distance_to_consider = 0.4;
        double robot_x;
        double robot_y;
};

#endif // FEATURE_EXTRACTOR_H