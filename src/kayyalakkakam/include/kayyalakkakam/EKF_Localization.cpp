#include "EKF_Localization.h"

EKF_Localization::EKF_Localization(ros::NodeHandle &N) : NH(N), robot(NH)
{
    ROS_INFO_STREAM("Initializing EKF NODE");
                                                                                    // Initialize the state vector and covariance matrix
    mu = Eigen::VectorXd(3);
    mu << 1.5, 1.5, 3.1415;                                                // Initiate State vector
    Sigma = Eigen::MatrixXd::Identity(3, 3);                                        // Initial covariance as Identity Matrix
    R = Eigen::MatrixXd::Identity(3, 3);                                            // Initial Process Noise Matrix as Identity Matrix
    g_function_jacobi = Eigen::MatrixXd::Identity(3, 3);                            // Initial Jacobian of the g_function
    detectCirclesInMap();                                                           // Extract Features from Map

                                                                                    // Initialize the Visualization publishers
    initializeEKFPublishers(NH);
    initializeMarkerPublisher(NH);
    initializeOdometryPublishers(NH);

    initializeEKFPosePublisher(NH);
};

/// EKF Pose Publisher ///

 void EKF_Localization::initializeEKFPosePublisher(ros::NodeHandle& nh)
 {
    EKF_Pose_Publihser = nh.advertise<nav_msgs::Path>("EKF_pose", 30, true);
 }


void EKF_Localization::publishEKFPose()
{
    geometry_msgs::PoseStamped ekf_pose;
    ekf_pose.header.stamp = ros::Time::now();
    ekf_pose.header.frame_id = "map";

    // Set the position from the EKF state vector
    ekf_pose.pose.position.x = mu(0);
    ekf_pose.pose.position.y = mu(1);
    ekf_pose.pose.position.z = 0;  

    // Set the orientation from the EKF state vector
    tf::Quaternion q = tf::createQuaternionFromYaw(mu(2));
    geometry_msgs::Quaternion q_msg;
    tf::quaternionTFToMsg(q, q_msg);
    ekf_pose.pose.orientation = q_msg;

    EKF_Pose_Publihser.publish(ekf_pose);
}

/// EKF Pose Publisher ///

/// Visualization Functions ///

void EKF_Localization::initializeEKFPublishers(ros::NodeHandle& nh) {
    EKF_path_pub = nh.advertise<nav_msgs::Path>("EKF_path", 30, true);
}

void EKF_Localization::publishEKFPath() {
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
    EKF_path_pub.publish(path);
}

void EKF_Localization::initializeOdometryPublishers(ros::NodeHandle& nh) {
    Odometry_path_pub = nh.advertise<nav_msgs::Path>("Odometry_path", 30, true);
}

void EKF_Localization::publishOdometryPath() {
    static nav_msgs::Path path;
    path.header.stamp = ros::Time::now();
    path.header.frame_id = "map"; // Or whatever your fixed frame is

    geometry_msgs::PoseStamped this_pose_stamped;
    this_pose_stamped.pose.position.x = this->odomMessage.pose.pose.position.x;
    this_pose_stamped.pose.position.y = this->odomMessage.pose.pose.position.y;
    this_pose_stamped.pose.orientation = this->odomMessage.pose.pose.orientation;
    this_pose_stamped.header.stamp = path.header.stamp;
    this_pose_stamped.header.frame_id = "map";

    path.poses.push_back(this_pose_stamped);
    Odometry_path_pub.publish(path);
}

void EKF_Localization::initializeMarkerPublisher(ros::NodeHandle& nh) {
    marker_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 10);
}

void EKF_Localization::publishCovariance() {
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
    marker.scale.x = 2 * 2 * sx;                                                    // 2 sigma for visualization
    marker.scale.y = 2 * 2 * sy;                                            
    marker.scale.z = 0.1;                                                           // Arbitrary small height for visibility

    marker.color.a = 0.3;                                                           // Transparency  
    marker.color.r = 0.0;
    marker.color.g = 0.0;
    marker.color.b = 1.0;                                                           // Blue

    marker_pub.publish(marker);
}

/// Visualization Functions ///

void EKF_Localization::printMuAndSigma()
{
    std::stringstream ss;
    ss << "New Mu: " << mu.transpose();                                             // Use transpose to print the vector in a row format
    ROS_INFO_STREAM(ss.str());                                                      // Logging the state vector

    ss.str("");                                                                     // Clearing the stringstream
    ss.clear();                                                                     // Clear any flags

                                                                                    // Assuming Sigma is not too large to print completely
    ss << "New Sigma: \n" << Sigma;                                 
    ROS_INFO_STREAM(ss.str());                                                      // Logging the covariance matrix

                                                                                    // Clear the stream after use
    ss.str("");
    ss.clear();
}

Eigen::VectorXd EKF_Localization::g_function(Eigen::VectorXd input_mu, nav_msgs::Odometry odometry)
{
    double v = odometry.twist.twist.linear.x;                                       // Linear velocity from odometry
    double omega = odometry.twist.twist.angular.z;                                  // Angular velocity from odometry
    double theta = input_mu(2);                                                     // Orientation from the state vector
    double dt = 0.0333;                                                              // Time step, set to odometry publishing frequency

    Eigen::VectorXd mu_delta(3);
    if (std::abs(omega) < 1e-6) {                                                   // Avoid division by zero by providing a small threshold
        mu_delta(0) = v * dt * cos(theta);  
        mu_delta(1) = v * dt * sin(theta);
        mu_delta(2) = 0;
    } else {
        mu_delta(0) = -(v/omega) * sin(theta) + (v/omega) * sin(theta + omega*dt);  // delta x
        mu_delta(1) = (v/omega) * cos(theta) - (v/omega) * cos(theta + omega*dt);   // delta y
        mu_delta(2) = omega * dt;                                                   // delta theta
    }                                                                               // delta theta

    mu_delta(2) = atan2(sin(mu_delta(2)), cos(mu_delta(2)));

    return mu_delta;
};

Eigen::MatrixXd EKF_Localization::updateSigma(Eigen::VectorXd input_mu, nav_msgs::Odometry odometry)
{
    double v = odometry.twist.twist.linear.x;                                       // Linear velocity from odometry
    double omega = odometry.twist.twist.angular.z;                                  // Angular velocity from odometry
    double theta = input_mu(2);                                                     // Orientation from the state vector
    double dt = 0.0333;                                                              // Time step, set to odometry publishing frequency

    theta = atan2(sin(theta), cos(theta));

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

void EKF_Localization::prediction_step()
{
    ROS_INFO_STREAM("PREDICTION RUNNING");
    this->odomMessage = this->robot.getOdom();

    mu = mu + g_function(mu, this->odomMessage);
    mu(2) = atan2(sin(mu(2)), cos(mu(2)));
    Sigma = updateSigma(mu, this->odomMessage);

    this->publishEKFPath();
    this->publishCovariance();
    this->publishOdometryPath();
    this->publishEKFPose();

    this->printMuAndSigma();
};

void EKF_Localization::correction_step()
{   
    // ROS_INFO_STREAM("CORRECTION RUNNING");
    this->laserscanMessage = this->robot.getLaserscan();

};

void EKF_Localization::run_EKF_Filter()
{
    this->prediction_step();
    this->correction_step();
}

/// Feature Extraction ///

void EKF_Localization::detectCirclesInMap()
{
    std::string map_path = "/home/cocokayya18/Probabilistic-Robotics-Lab/src/kayyalakkakam/src/map/turtlebot3_world_map.pgm";
    cv::Mat map_image = cv::imread(map_path, cv::COLOR_BGR2GRAY);
    cv::Mat gaussianImg;
    cv::Mat treshImg;
    cv::Mat cannyImg;
    cv::Mat colorImage;

    if (map_image.empty()) {
        std::cerr << "Failed to load image at path: " << map_path << std::endl;
        return;
    }

    cv::cvtColor(map_image, colorImage, cv::COLOR_GRAY2BGR);

    cv::imwrite("/home/cocokayya18/Probabilistic-Robotics-Lab/src/kayyalakkakam/src/map/Normal_Picture.jpg", map_image);


    cv::GaussianBlur(map_image, gaussianImg, cv::Size(9,9), 3, 3);
    cv::imwrite("/home/cocokayya18/Probabilistic-Robotics-Lab/src/kayyalakkakam/src/map/Gaussian_Picture.jpg", gaussianImg);

    cv::threshold(gaussianImg, treshImg,0,255,cv::THRESH_BINARY+cv::THRESH_OTSU);
    cv::imwrite("/home/cocokayya18/Probabilistic-Robotics-Lab/src/kayyalakkakam/src/map/Treshholded_Picture.jpg", treshImg);

    cv::Canny(treshImg, cannyImg, 50, 150);
    cv::imwrite("/home/cocokayya18/Probabilistic-Robotics-Lab/src/kayyalakkakam/src/map/Canny_Picture.jpg", cannyImg);

    std::vector<cv::Vec3f> circles;
    cv::HoughCircles(cannyImg, circles, cv::HOUGH_GRADIENT, 1, cannyImg.rows/32, 100, 12, 1, 7);

     for( size_t i = 0; i < circles.size(); i++ )
    {
        cv::Vec3i c = circles[i];
        Circle detectedCircle;
        cv::Point center = cv::Point(c[0], c[1]);
        int radius = c[2];
        
        circle( colorImage, center, 1, cv::Scalar(0,255,255), 0.1, cv::LINE_AA);
        circle( colorImage, center, radius, cv::Scalar(0,255,255), 0.1, cv::LINE_AA);
        
        detectedCircle.center = cv::Point(c[0], c[1]);
        detectedCircle.radius = c[2];
        detectedCirclesInMap.push_back(detectedCircle);
    }

    imwrite("/home/cocokayya18/Probabilistic-Robotics-Lab/src/kayyalakkakam/src/map/Map_With_Circles.jpg", colorImage);
    
    for ( int features = 0; features < detectedCirclesInMap.size(); features++)
    {
        WorldCoords world;
        world.x = origin.x + detectedCirclesInMap[features].center.x * resolution;
        world.y = origin.y + detectedCirclesInMap[features].center.y * resolution;
        mapFeatures.push_back(world);   

        std::string filename = "/home/cocokayya18/Probabilistic-Robotics-Lab/src/kayyalakkakam/src/features/features.csv";

        std::ofstream out_file(filename);
        if (!out_file) {
            std::cerr << "Failed to open file for writing.\n";
            return;
        }

        // Set precision for better accuracy
        out_file << std::fixed << std::setprecision(5);

        for (const auto& coords : mapFeatures) {
            out_file << coords.x << ", " << coords.y << '\n';
        }

        out_file.close();
        ROS_INFO_STREAM("Features saved in world coordinates to " << filename << '\n');
    }
}

void EKF_Localization::LidarToImage()
{

}

void EKF_Localization::detectCircleInLidar()
{
    
}

/// Feature Extraction ///