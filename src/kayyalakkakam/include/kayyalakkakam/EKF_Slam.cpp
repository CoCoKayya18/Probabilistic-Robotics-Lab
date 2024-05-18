#include "EKF_Slam.h"

EKF_Slam::EKF_Slam(ros::NodeHandle &N) : NH(N), robot(NH)
{
    ROS_INFO_STREAM("Initializing EKF NODE");
    // Initialize the state vector and covariance matrix
    mu = Eigen::VectorXd(6);
    mu << 1.5, 1.5, 3.1415, 0, 0, 0;                        // Initiate State vector
    Sigma = Eigen::MatrixXd::Identity(6, 6);                // Initial covariance as Identity Matrix
    R = Eigen::MatrixXd::Identity(6, 6);                    // Initial Process Noise Matrix as Identity Matrix
    g_function_jacobi = Eigen::MatrixXd::Identity(6, 6);    // Initial Jacobian of the g_function

    // Initialize the Visualization publishers
    initializePublishers(NH);
    initializeMarkerPublisher(NH);
};

/// Visualization Functions ///

void EKF_Slam::initializePublishers(ros::NodeHandle& nh) {
    path_pub = nh.advertise<nav_msgs::Path>("EKF_path", 10, true);
}

void EKF_Slam::publishPath() {
    static nav_msgs::Path path;
    path.header.stamp = ros::Time::now();
    path.header.frame_id = "map"; // Or whatever your fixed frame is

    geometry_msgs::PoseStamped this_pose_stamped;
    this_pose_stamped.pose.position.x = mu(0);
    this_pose_stamped.pose.position.y = mu(1);
    this_pose_stamped.pose.orientation = tf::createQuaternionMsgFromYaw(mu(2));
    this_pose_stamped.header.stamp = path.header.stamp;
    this_pose_stamped.header.frame_id = "map";

    path.poses.push_back(this_pose_stamped);
    path_pub.publish(path);
}

void EKF_Slam::initializeMarkerPublisher(ros::NodeHandle& nh) {
    marker_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 10);
}

void EKF_Slam::publishCovariance() {
    visualization_msgs::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();
    marker.ns = "EKF";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = mu(0);
    marker.pose.position.y = mu(1);
    marker.pose.position.z = 0;

    // Assuming your covariance matrix is 6x6 and corresponds to x, y, theta
    // Extract the covariance for x and y
    double sx = sqrt(Sigma(0, 0));
    double sy = sqrt(Sigma(1, 1));
    marker.scale.x = 2 * 2 * sx; // 2 sigma for visualization
    marker.scale.y = 2 * 2 * sy;
    marker.scale.z = 0.1; // Arbitrary small height for visibility

    marker.color.a = 0.3; // Transparency
    marker.color.r = 0.0;
    marker.color.g = 0.0;
    marker.color.b = 1.0; // Blue

    marker_pub.publish(marker);
}

/// Visualization Functions ///

void EKF_Slam::printMuAndSigma()
{
    std::stringstream ss;
    ss << "New Mu: " << mu.transpose();  // Use transpose to print the vector in a row format
    ROS_INFO_STREAM(ss.str());  // Logging the state vector

    ss.str("");  // Clearing the stringstream
    ss.clear();  // Clear any flags

    // Assuming Sigma is not too large to print completely
    ss << "New Sigma: \n" << Sigma;
    ROS_INFO_STREAM(ss.str());  // Logging the covariance matrix

    // Clear the stream after use
    ss.str("");
    ss.clear();
}

Eigen::VectorXd EKF_Slam::g_function(Eigen::VectorXd input_mu, nav_msgs::Odometry odometry)
{
    double v = odometry.twist.twist.linear.x;  // Linear velocity from odometry
    double omega = odometry.twist.twist.angular.z;  // Angular velocity from odometry
    double theta = input_mu(2);  // Orientation from the state vector
    double dt = 0.333;  // Time step, set to odometry publishing frequency

    Eigen::VectorXd mu_delta(6);
    if (std::abs(omega) < 1e-6) {                                                   // Avoid division by zero by providing a small threshold
        mu_delta(0) = v * dt * cos(theta);  
        mu_delta(1) = v * dt * sin(theta);
        mu_delta(2) = 0;
    } else {
        mu_delta(0) = -(v/omega) * sin(theta) + (v/omega) * sin(theta + omega*dt);  // delta x
        mu_delta(1) = (v/omega) * cos(theta) - (v/omega) * cos(theta + omega*dt);   // delta y
        mu_delta(2) = omega * dt;                                                   // delta theta
    }                                                                               // delta theta
    mu_delta(3) = odometry.twist.twist.linear.x;
    mu_delta(4) = 0;
    mu_delta(5) = odometry.twist.twist.angular.z;

    return mu_delta;
};

Eigen::MatrixXd EKF_Slam::updateSigma(Eigen::VectorXd input_mu, nav_msgs::Odometry odometry)
{
    double v = odometry.twist.twist.linear.x;       // Linear velocity from odometry
    double omega = odometry.twist.twist.angular.z;  // Angular velocity from odometry
    double theta = input_mu(2);                     // Orientation from the state vector
    double dt = 0.333;                              // Time step, set to odometry publishing frequency

    Eigen::MatrixXd newSigma;
    
    if (std::abs(omega) < 1e-6) {
        g_function_jacobi(0, 2) = -v * dt * sin(theta);
        g_function_jacobi(1, 2) = v * dt * cos(theta);
    } else {
        g_function_jacobi(0, 2) = -v/omega * cos(theta) + v/omega * cos(theta + omega*dt);
        g_function_jacobi(1, 2) = -v/omega * sin(theta) + v/omega * sin(theta + omega*dt);
    }

    newSigma = g_function_jacobi * this->Sigma * g_function_jacobi.transpose() + this->R;

    return newSigma;
}

void EKF_Slam::prediction_step()
{
    ROS_INFO_STREAM("PREDICTION RUNNING");
    this->odomMessage = this->robot.getOdom();
    mu = mu + g_function(mu, this->odomMessage);
    Sigma = updateSigma(mu, this->odomMessage);

    this->publishPath();
    this->publishCovariance();

    this->printMuAndSigma();
};

void EKF_Slam::correction_step()
{   
    // ROS_INFO_STREAM("CORRECTION RUNNING");
    this->odomMessage = this->robot.getOdom();

};

void EKF_Slam::run_EKF_Filter()
{
    this->prediction_step();
    this->correction_step();
}