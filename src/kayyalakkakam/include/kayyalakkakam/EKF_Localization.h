#include <vector>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_broadcaster.h>
#include <opencv2/opencv.hpp>
#include <tf/transform_listener.h>
#include <laser_geometry/laser_geometry.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_circle.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include "robot_class.h"

struct Circle {
    cv::Point2f center;
    float radius;
};

struct WorldCoords {
    double x;
    double y;
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

        void H_Function();

        /// Visualization Functions ///
        void publishEKFPath();
        void publishCovariance();
        void publishOdometryPath();
        void publishRansacFeatures();
        /// Visualization Functions ///

        /// Feature Extraction ///
        void detectCirclesInMap();
        void detectCircleInLidar(sensor_msgs::LaserScan input);
        /// Feature Extraction ///

        /// EKF Pose Publisher /// 
        void publishEKFPose();
        /// EKF Pose Publisher /// 

    private:
        ros::NodeHandle NH;
        Ros_Subscriber_Publisher_Class robot;
        nav_msgs::Odometry odomMessage;
        sensor_msgs::LaserScan laserscanMessage;
        Eigen::VectorXd mu; // State vector: [x, y, theta, vx, vy, vtheta]
        Eigen::MatrixXd Sigma; // Covariance matrix
        Eigen::MatrixXd g_function_jacobi;
        Eigen::MatrixXd R; // Process Noise
        Eigen::MatrixXd Q; // Measurement Noise
        float sensor_noise = 0.1;

        // ros::Publisher EKF_path_pub;    // EKF Path publisher 
        // ros::Publisher Odometry_path_pub;    // EKF Path publisher 
        // ros::Publisher marker_pub;  // Covariance Publisher
        // ros::Publisher EKF_Pose_Publihser;

        std::vector<Circle> detectedCirclesInMap;
        std::vector<Circle> detectedCirclesInLidar;
        std::vector<WorldCoords> mapFeatures;
        // std::vector<pcl::ModelCoefficients> lidarFeatures;
        tf::TransformListener transformer;
        laser_geometry::LaserProjection projector_;
        double resolution = 0.050000;
        WorldCoords origin = {-10.0, -10.0};
};