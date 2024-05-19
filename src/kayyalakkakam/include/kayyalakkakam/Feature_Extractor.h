#ifndef FEATURE_EXTRACTOR_H
#define FEATURE_EXTRACTOR_H

#include <vector>
#include <string>
#include <cmath>
#include <fstream>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <laser_geometry/laser_geometry.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/centroid.h>

class FeatureExtractor 
{
    public:
        FeatureExtractor(ros::NodeHandle& nh);
        void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan);

    private:
        ros::NodeHandle nh_;
        ros::Subscriber scan_sub_;
        ros::Publisher features_marker_pub_;
        std::vector<std::pair<double, double>> features_;
        
        laser_geometry::LaserProjection projector_;  // To convert LaserScan to PointCloud
        tf::TransformListener transformer;
        void detectFeatures(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);
        void saveFeaturesToFile(const std::string& filename);
        void publishMarkers();
};

#endif // FEATURE_EXTRACTOR_H