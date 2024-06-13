#include <vector>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_broadcaster.h>
#include <opencv2/opencv.hpp>
#include <tf/transform_listener.h>
#include <cmath>
#include <laser_geometry/laser_geometry.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_circle.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/io/pcd_io.h>
#include <pcl/features/integral_image_normal.h> 
#include <pcl/features/normal_3d.h> 
#include <pcl/features/boundary.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/visualization/pcl_visualizer.h>
#include "robot_class.h"

struct Circle {
    cv::Point2f center;
    float radius;
};

struct WorldCoords {
    double x;
    double y;
    double radius;
};

class EKF_Localization
{
    public:
        EKF_Localization(ros::NodeHandle &N);
        ~EKF_Localization(){};


        Eigen::VectorXd g_function(Eigen::VectorXd input_mu, nav_msgs::Odometry odometry);
        Eigen::MatrixXd updateSigma(Eigen::VectorXd input_mu, nav_msgs::Odometry odometry);

        void prediction_step();
        void correction_step();

        void run_EKF_Filter();
        void printMuAndSigma();

        void h_Function(std::vector<int> Indices);
        std::vector<int> landmarkMatching(const std::vector<Circle>& detectedFeatures);

        /// Visualization Functions ///
        void publishEKFPath();
        void publishCovariance();
        void publishOdometryPath();
        void publishRansacFeatures();
        /// Visualization Functions ///

        /// Feature Extraction ///
        void detectCirclesInMap();
        void detectCircleInLidar(sensor_msgs::LaserScan input);
        void detectCornersInLidar(sensor_msgs::LaserScan input);
        /// Feature Extraction ///

        /// EKF Pose Publisher /// 
        void publishEKFPose();
        /// EKF Pose Publisher /// 

    private:
        ros::NodeHandle NH;
        Ros_Subscriber_Publisher_Class robot;
        nav_msgs::Odometry odomMessage;
        sensor_msgs::LaserScan laserscanMessage;

        Eigen::VectorXd mu; // State vector: [x, y, theta]
        Eigen::MatrixXd Sigma; // Covariance matrix
        Eigen::MatrixXd g_function_jacobi;

        Eigen::VectorXd delta;
        double q;
        Eigen::VectorXd z_hat;
        Eigen::VectorXd z_polar;
        Eigen::Vector3d z_difference;
        Eigen::MatrixXd h_function_jacobi;
        Eigen::MatrixXd KalmanGain;

        Eigen::VectorXd Kalman_Sensor_Sum_for_Mu;
        Eigen::MatrixXd Kalman_H_Matrix_Sigma_Sum_For_Sigma;

        Eigen::MatrixXd R; // Process Noise
        float process_noise = 0.1;
        Eigen::MatrixXd Q; // Measurement Noise
        float sensor_noise = 0.1;

        std::vector<Circle> detectedCirclesInMap;
        std::vector<Circle> detectedCirclesInLidar;
        std::vector<WorldCoords> mapFeatures;
        pcl::PointCloud<pcl::PointXYZ>::Ptr map_cloud;
        pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
        // std::vector<pcl::ModelCoefficients> lidarFeatures;
        tf::TransformListener transformer;
        laser_geometry::LaserProjection projector_;
        double resolution = 0.050000;
        WorldCoords origin = {-10.0, -10.0};
        const int maxRansacFeatureSize = 9;
        const float NewFeatureThreshold = 0.5;
};