#ifndef EKF_LOCALIZATION_H
#define EKF_LOCALIZATION_H

#include <vector>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>
#include <tf/transform_broadcaster.h>
#include <opencv2/opencv.hpp>
#include <tf/transform_listener.h>
#include <cmath>
#include <laser_geometry/laser_geometry.h>
#include <pcl/kdtree/kdtree_flann.h>

struct point
{
    double x;
    double y;
};

class EKF_Localization
{
    public:
        EKF_Localization();
        ~EKF_Localization(){};


        Eigen::VectorXd g_function(Eigen::VectorXd input_mu, nav_msgs::Odometry odometry);
        Eigen::MatrixXd updateSigma(Eigen::VectorXd input_mu, nav_msgs::Odometry odometry);

        void prediction_step(nav_msgs::Odometry odomMsg);
        void correction_step(sensor_msgs::LaserScan laserscanMsg);

        // void run_EKF_Filter();
        void printMuAndSigma();

        void h_Function(std::vector<int> Indices);
        std::vector<int> landmarkMatching(const std::vector<point>& detectedFeatures);

        /// Visualization Functions ///
        nav_msgs::Path publishEKFPath();
        visualization_msgs::Marker publishCovariance();
        nav_msgs::Path publishOdometryPath();
        visualization_msgs::Marker publishCornerFeatures();
        /// Visualization Functions ///

        /// Feature Extraction ///
        void detectCornersInMap();
        void detectCornersInLidar(sensor_msgs::LaserScan input);
        std::vector<point> detectCorners(const std::vector<point>& points);
        /// Feature Extraction ///

        /// EKF Pose Publisher /// 
        geometry_msgs::PoseStamped publishEKFPose();
        /// EKF Pose Publisher /// 

    private:
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
        float process_noise = 0.01;
        Eigen::MatrixXd Q; // Measurement Noise
        float sensor_noise = 0.01;

        std::vector<point> cornersInLidar;
        std::vector<point> mapCornerFeatures;
        pcl::PointCloud<pcl::PointXYZ>::Ptr map_Corner_Cloud;
        pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
        tf::TransformListener transformer;
        laser_geometry::LaserProjection projector_;
        double resolution = 0.050000;
};

#endif // EKF_LOCALIZATION_H