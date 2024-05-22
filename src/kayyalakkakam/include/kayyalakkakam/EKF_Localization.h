#include <vector>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>
#include <tf/transform_broadcaster.h>
#include <opencv2/opencv.hpp>
#include "robot_class.h"

struct Circle {
    cv::Point center;
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

        /// Visualization Functions ///
        void initializeEKFPublishers(ros::NodeHandle& nh);
        void publishEKFPath();
        void initializeMarkerPublisher(ros::NodeHandle& nh);
        void publishCovariance();
        void initializeOdometryPublishers(ros::NodeHandle& nh);
        void publishOdometryPath();
        /// Visualization Functions ///

        /// Feature Extraction ///
        void detectCirclesInMap();
        void LidarToImage();
        void detectCircleInLidar();
        /// Feature Extraction ///

        /// EKF Pose Publisher /// 
        void initializeEKFPosePublisher(ros::NodeHandle& nh);
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

        ros::Publisher EKF_path_pub;    // EKF Path publisher 
        ros::Publisher Odometry_path_pub;    // EKF Path publisher 
        ros::Publisher marker_pub;  // Covariance Publisher
        ros::Publisher EKF_Pose_Publihser;

        std::vector<Circle> detectedCirclesInMap;
        std::vector<Circle> detectedCirclesInLidar;
        std::vector<WorldCoords> mapFeatures;
        double resolution = 0.050000;
        WorldCoords origin = {-10.0, -10.0};
};