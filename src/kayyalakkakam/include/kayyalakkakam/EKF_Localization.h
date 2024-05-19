#include <vector>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>
#include <tf/transform_broadcaster.h>
#include "robot_class.h"

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
};