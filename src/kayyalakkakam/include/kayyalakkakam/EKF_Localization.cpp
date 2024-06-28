#include "EKF_Localization.h"

EKF_Localization::EKF_Localization(ros::NodeHandle &N) : NH(N), robot(NH)
{
    ROS_INFO_STREAM("Initializing EKF NODE");
                                                                                    // Initialize the state vector and covariance matrix
    mu = Eigen::VectorXd(3);                                                        // Initiate State vector
    mu << 1.5, 1.5, 3.1415; 
    delta = Eigen::VectorXd(3);
    z_hat = Eigen::VectorXd(3);
    z_polar = Eigen::VectorXd(3);
    z_difference = Eigen::VectorXd(3);                                                     
    Sigma = Eigen::MatrixXd::Identity(3, 3);                                        // Initial covariance as Identity Matrix
    R = Eigen::MatrixXd::Identity(3, 3);                                            // Initial Process Noise Matrix as Identity Matrix
    R << process_noise, 0, 0, 0, process_noise, 0, 0, 0, process_noise;
    Q = Eigen::MatrixXd::Identity(3,3);
    Q << sensor_noise, 0, 0, 0, sensor_noise, 0, 0, 0, 1;
    KalmanGain = Eigen::MatrixXd::Identity(3,3);
    g_function_jacobi = Eigen::MatrixXd::Identity(3, 3);                            // Initial Jacobian of the g_function
    h_function_jacobi = Eigen::MatrixXd::Identity(3, 3);                            // Initial Jacobian of the h_function
    Kalman_Sensor_Sum_for_Mu = Eigen::VectorXd(3);
    Kalman_H_Matrix_Sigma_Sum_For_Sigma = Eigen::MatrixXd::Identity(3, 3); 
    // map_Circle_Cloud.reset(new pcl::PointCloud<pcl::PointXYZ>());                          // Initialize Space for map feature point cloud
    // detectCirclesInMap();                                                           // Extract Circle Features from Map
    map_Corner_Cloud.reset(new pcl::PointCloud<pcl::PointXYZ>());
    detectCornersInMap();
};

/// EKF Pose Publisher ///

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

    robot.publishEKFPose(ekf_pose);

}

/// EKF Pose Publisher ///


/// Visualization Functions ///

void EKF_Localization::publishEKFPath() 
{
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

    robot.publishEKFpath(path);

}

void EKF_Localization::publishOdometryPath() 
{
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

    robot.publishOdometryPath(path);
}

void EKF_Localization::publishCovariance() 
{
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

    double sx = sqrt(Sigma(0, 0));                                                  // Assuming your covariance matrix is 6x6 and corresponds to x, y, theta
    double sy = sqrt(Sigma(1, 1));                                          
    marker.scale.x = 2 * 2 * sx;                                                    // 2 sigma for visualization
    marker.scale.y = 2 * 2 * sy;                                            
    marker.scale.z = 0.1;                                                           // Arbitrary small height for visibility

    marker.color.a = 0.3;                                                           // Transparency  
    marker.color.r = 0.0;
    marker.color.g = 0.0;
    marker.color.b = 1.0;                                                           // Blue

    robot.publishCovariance(marker);
}

// void EKF_Localization::publishRansacFeatures()
// {
//     // ROS_INFO_STREAM("Publishing Ransac Feature Marker Array");

//     visualization_msgs::MarkerArray markerArray;

//     for (size_t i = 0; i < detectedCirclesInLidar.size(); i++) {
//         const auto& feature = detectedCirclesInLidar[i];
        
//         visualization_msgs::Marker marker;
//         marker.header.frame_id = "map";  // Ensure this matches your coordinate system in RViz
//         marker.header.stamp = ros::Time::now();
//         marker.ns = "circles";
//         marker.id = i;
//         marker.type = visualization_msgs::Marker::CYLINDER;
//         marker.action = visualization_msgs::Marker::ADD;

//         marker.pose.position.x = feature.center.x;
//         marker.pose.position.y = feature.center.y;
//         marker.pose.position.z = 0;  
//         marker.pose.orientation.x = 0.0;
//         marker.pose.orientation.y = 0.0;
//         marker.pose.orientation.z = 0.0;
//         marker.pose.orientation.w = 1.0;

//         marker.scale.x = feature.radius * 2;  
//         marker.scale.y = feature.radius * 2;  
//         marker.scale.z = 0.1;                 
//         marker.color.r = 1.0;
//         marker.color.g = 0.0;
//         marker.color.b = 1.0;  // Purple for visualization
//         marker.color.a = 0.8;  

//         marker.lifetime = ros::Duration();  

//         markerArray.markers.push_back(marker);
//     }

//     robot.publishRansacFeatures(markerArray);
// }

void EKF_Localization::publishCornerFeatures()
{
    visualization_msgs::Marker points;
    points.header.frame_id = "map";
    points.header.stamp = ros::Time::now();
    points.ns = "corners";
    points.action = visualization_msgs::Marker::ADD;
    points.pose.orientation.w = 1.0;
    points.id = 0;
    points.type = visualization_msgs::Marker::POINTS;

    // Set the scale of the points
    points.scale.x = 0.2; // Point width
    points.scale.y = 0.2; // Point height

    // Set the color of the points
    points.color.r = 0.0f;
    points.color.g = 1.0f;
    points.color.b = 0.0f;
    points.color.a = 1.0;

    for (const auto& corner : cornersInLidar) {
        geometry_msgs::Point p;
        p.x = corner.x;
        p.y = corner.y;
        p.z = 0; 

        points.points.push_back(p);
    }

    robot.publishCornerFeatures(points);
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

void EKF_Localization::h_Function(std::vector<int> Indices)
{
    for (size_t i = 0; i < cornersInLidar.size(); ++i) {

        if (Indices[i] != -1) { 

            auto detectedFeature = cornersInLidar[i];
            auto mapFeature = mapCornerFeatures[Indices[i]];

            // std::cout << "Matching map feature X: " << mapFeature.x << " Y: " << mapFeature.y << std::endl;
            // std::cout << "Matching lidar feature X: " << detectedFeature.x << " Y: " << detectedFeature.y << std::endl;

            z_polar << std::sqrt(std::pow(detectedFeature.x, 2) + std::pow(detectedFeature.y, 2)), std::atan2(detectedFeature.y, detectedFeature.x), 0;            

            delta(0) = mapFeature.x - mu(0);
            delta(1) = mapFeature.y - mu(1);

            q = delta.squaredNorm();

            z_hat << std::sqrt(q),
                     std::atan2(delta.y(), delta.x()) - mu(2),
                     0;

            h_function_jacobi << delta.x() / std::sqrt(q), - delta.y() / std::sqrt(q), 0,
                                 delta.y() / q, delta.x() / q, -1/q,
                                 0, 0, 0;

            // h_function_jacobi = h_function_jacobi.array().isNaN().select(0, h_function_jacobi);

            Eigen::MatrixXd S = h_function_jacobi * Sigma * h_function_jacobi.transpose() + Q;
            KalmanGain = Sigma * h_function_jacobi.transpose() * S.inverse();

            z_difference = z_polar - z_hat;

            Kalman_Sensor_Sum_for_Mu = Kalman_Sensor_Sum_for_Mu + KalmanGain * z_difference;
            Kalman_H_Matrix_Sigma_Sum_For_Sigma = Kalman_H_Matrix_Sigma_Sum_For_Sigma + (Eigen::MatrixXd::Identity(mu.size(), mu.size()) - KalmanGain * h_function_jacobi);

            ROS_INFO_STREAM("Debug Information: \n");
            ROS_INFO_STREAM("Current Index: " << Indices[i] << "\n");
            ROS_INFO_STREAM("Map Feature: x=" << mapFeature.x << ", y=" << mapFeature.y);
            ROS_INFO_STREAM("Detected Feature: x=" << detectedFeature.x << ", y=" << detectedFeature.y);
            ROS_INFO_STREAM("Delta: [" << delta(0) << ", " << delta(1) << "]");
            ROS_INFO_STREAM("q (squared norm of delta): " << q);
            ROS_INFO_STREAM("z_hat: [" << z_hat(0) << ", " << z_hat(1) << ", " << z_hat(2) << "]");
            ROS_INFO_STREAM("Measurement Residual (z_difference): [" << z_difference(0) << ", " << z_difference(1) << ", " << z_difference(2) << "]\n");
            ROS_INFO_STREAM("Measurement Function Jacobian (H): \n" << h_function_jacobi << "\n");
            ROS_INFO_STREAM("S-Matrix: \n" << S << "\n");
            ROS_INFO_STREAM("Kalman Gain (K): \n" << KalmanGain);
            ROS_INFO_STREAM("Measurement Residual (z_difference): [" << z_difference(0) << ", " << z_difference(1) << ", " << z_difference(2) << "]");
            ROS_INFO_STREAM("Kalman Gain weighted sum for mu update (Kalman_Sensor_Sum_for_Mu): \n" << Kalman_Sensor_Sum_for_Mu << "\n");
            ROS_INFO_STREAM("Sigma update sum part (Kalman_H_Matrix_Sigma_Sum_For_Sigma): \n" << Kalman_H_Matrix_Sigma_Sum_For_Sigma << "\n");


        } 

        // else {
        //     ROS_WARN_STREAM("No valid match for detected feature");
        // }        
    }

    // ROS_INFO_STREAM("Debug Information: \n");
    // ROS_INFO_STREAM("Delta: [" << delta(0) << ", " << delta(1) << "]\n");
    // ROS_INFO_STREAM("q (squared norm of delta): " << q << "\n");
    // ROS_INFO_STREAM("Predicted Measurement (z_hat): [" << z_hat(0) << ", " << z_hat(1) << ", " << z_hat(2) << "]\n");
    // ROS_INFO_STREAM("Measurement Function Jacobian (H): \n" << h_function_jacobi << "\n");
    // ROS_INFO_STREAM("Kalman Gain (K): \n" << KalmanGain << "\n");
    // ROS_INFO_STREAM("Measurement Residual (z_difference): [" << z_difference(0) << ", " << z_difference(1) << ", " << z_difference(2) << "]\n");
    // ROS_INFO_STREAM("Mu before: " << mu);
    // ROS_INFO_STREAM("Sigma before: " << Sigma);
    // ROS_INFO_STREAM("Kalman Gain weighted sum for mu update (Kalman_Sensor_Sum_for_Mu): \n" << Kalman_Sensor_Sum_for_Mu << "\n");
    // ROS_INFO_STREAM("Sigma update sum part (Kalman_H_Matrix_Sigma_Sum_For_Sigma): \n" << Kalman_H_Matrix_Sigma_Sum_For_Sigma << "\n");

    mu = mu + Kalman_Sensor_Sum_for_Mu;
    Sigma = Kalman_H_Matrix_Sigma_Sum_For_Sigma * Sigma; 

    // ROS_INFO_STREAM("Mu after: " << mu);
    // ROS_INFO_STREAM("Sigma after: " << Sigma);

    // if (!Kalman_H_Matrix_Sigma_Sum_For_Sigma.isApprox(Eigen::MatrixXd::Identity(mu.size(), mu.size()))) {
    //     ROS_ERROR_STREAM("Kalman_H_Matrix_Sigma_Sum_For_Sigma is not the identity matrix. Stopping the program.");
    //     ROS_INFO_STREAM("Sigma update sum part (Kalman_H_Matrix_Sigma_Sum_For_Sigma): \n" << Kalman_H_Matrix_Sigma_Sum_For_Sigma << "\n");
    //     ros::shutdown();
    //     return;
    // }

    // // Check if Kalman_Sensor_Sum_for_Mu has any non-zero values
    // if (Kalman_Sensor_Sum_for_Mu.array().abs().sum() > 0.05) {
    //     ROS_ERROR_STREAM("Kalman_Sensor_Sum_for_Mu has non-zero values. Stopping the program.");
    //     ROS_INFO_STREAM("Kalman Gain weighted sum for mu update (Kalman_Sensor_Sum_for_Mu): \n" << Kalman_Sensor_Sum_for_Mu << "\n");
    //     ros::shutdown();
    //     return;
    // }

    // Reset the Sums
    Kalman_Sensor_Sum_for_Mu.setZero(3);
    Kalman_H_Matrix_Sigma_Sum_For_Sigma = Eigen::MatrixXd::Identity(3, 3);

};

std::vector<int> EKF_Localization::landmarkMatching(const std::vector<point>& detectedFeatures)
{

    if (!map_Corner_Cloud) {
        ROS_ERROR("map_Corner_Cloud is not initialized.");
        return std::vector<int>();
    }

    kdtree.setInputCloud(map_Corner_Cloud);

    // ROS_INFO_STREAM("Number of map corner features: " << detectedFeatures.size());
    // ROS_INFO_STREAM("Number of points in map cloud: " << map_Corner_Cloud->points.size());

    std::vector<int> indices(detectedFeatures.size(), -1);
    double threshold = 0.1;

        for (size_t i = 0; i < detectedFeatures.size(); ++i) {
            pcl::PointXYZ searchPoint(detectedFeatures[i].x, detectedFeatures[i].y, 0);
            std::vector<int> pointIdxNKNSearch(1);
            std::vector<float> pointNKNSquaredDistance(1);

            if (kdtree.nearestKSearch(searchPoint, 1, pointIdxNKNSearch, pointNKNSquaredDistance) > 0) {
                
                if (pointNKNSquaredDistance[0] <= threshold * threshold) {
                    indices[i] = pointIdxNKNSearch[0];
                    ROS_INFO_STREAM("Matched detected feature (" << detectedFeatures[i].x << ", " << detectedFeatures[i].y << ") to map feature index " << pointIdxNKNSearch[0]);
                } 
                
                else {
                    // ROS_WARN_STREAM("No match within threshold for detected feature (" << detectedFeatures[i].x << ", " << detectedFeatures[i].y << ")");
                } 
            }
            
            else {
                // ROS_WARN_STREAM("No nearest neighbor found for detected feature (" << detectedFeatures[i].x << ", " << detectedFeatures[i].y << ")");
            }
        }

        return indices;
};

void EKF_Localization::prediction_step()
{
    // ROS_INFO_STREAM("PREDICTION RUNNING");
    this->odomMessage = this->robot.getOdom();

    mu = mu + g_function(mu, this->odomMessage);
    mu(2) = atan2(sin(mu(2)), cos(mu(2)));
    Sigma = updateSigma(mu, this->odomMessage);

    // this->publishEKFPath();
    // this->publishCovariance();
    // this->publishEKFPose();
    // this->publishOdometryPath();

    // this->printMuAndSigma();
};

void EKF_Localization::correction_step()
{   
    // ROS_INFO_STREAM("CORRECTION RUNNING");
    this->laserscanMessage = this->robot.getLaserscan();

    detectCornersInLidar(laserscanMessage);

    std::vector<int> matchedIndices = landmarkMatching(this->cornersInLidar);

    // Process each matched feature
    h_Function(matchedIndices);
    publishCornerFeatures();
    // publishRansacFeatures();
    publishEKFPath();
    publishCovariance();
    publishEKFPose();
    publishOdometryPath();
    // printMuAndSigma();
};

void EKF_Localization::run_EKF_Filter()
{
    this->prediction_step();
    this->correction_step();
};

/// Feature Extraction ///


void EKF_Localization::detectCornersInMap()
{
    std::string map_path = "/home/cocokayya18/Probabilistic-Robotics-Lab/src/kayyalakkakam/src/map/turtlebot3_world_map.pgm";
    cv::Mat map_image = cv::imread(map_path, cv::COLOR_BGR2GRAY);
    cv::Mat gaussianImg;
    cv::Mat treshImg;
    cv::Mat cannyImg;
    cv::Mat colorImage;
    cv::Mat harrisImg;
    cv::Mat harris_norm;
    cv::Mat harris_norm_scaled;
    cv::Mat greyErodedContourImage;
    cv::Mat sharpened_image;
    cv::Mat eroded_image;
    std::string filename;

    if (map_image.empty()) {
        std::cerr << "Failed to load image at path: " << map_path << std::endl;
        return;
    }

    cv::cvtColor(map_image, colorImage, cv::COLOR_GRAY2BGR);

    cv::imwrite("/home/cocokayya18/Probabilistic-Robotics-Lab/src/kayyalakkakam/src/map/Altered_MapImages/Normal_Picture.jpg", map_image);


    cv::GaussianBlur(map_image, gaussianImg, cv::Size(9,9), 3, 3);
    cv::imwrite("/home/cocokayya18/Probabilistic-Robotics-Lab/src/kayyalakkakam/src/map/Altered_MapImages/Gaussian_Picture.jpg", gaussianImg);

    cv::threshold(gaussianImg, treshImg,0,255,cv::THRESH_BINARY+cv::THRESH_OTSU);
    cv::imwrite("/home/cocokayya18/Probabilistic-Robotics-Lab/src/kayyalakkakam/src/map/Altered_MapImages/Treshholded_Picture.jpg", treshImg);

    cv::Canny(treshImg, cannyImg, 50, 150);
    cv::imwrite("/home/cocokayya18/Probabilistic-Robotics-Lab/src/kayyalakkakam/src/map/Altered_MapImages/Canny_Picture.jpg", cannyImg);

    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(cannyImg, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    
    cv::Mat contourImage = cv::Mat::zeros(cannyImg.size(), CV_8UC3); 
    if (!contours.empty()) {
        // Draw only the largest contour
        auto largestContour = std::max_element(contours.begin(), contours.end(), [](const std::vector<cv::Point>& c1, const std::vector<cv::Point>& c2) {
            return cv::contourArea(c1) < cv::contourArea(c2);
        });
        cv::drawContours(contourImage, std::vector<std::vector<cv::Point>>{*largestContour}, -1, cv::Scalar(0, 255, 0), 2);
    }

    // Save the resulting image with emphasized outer edge
    cv::imwrite("/home/cocokayya18/Probabilistic-Robotics-Lab/src/kayyalakkakam/src/map/Altered_MapImages/Contour_Picture.jpg", contourImage);

    cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
    cv::erode(contourImage, eroded_image, element);

    cv::imwrite("/home/cocokayya18/Probabilistic-Robotics-Lab/src/kayyalakkakam/src/map/Altered_MapImages/Erored_Picture.jpg", eroded_image);

    cv::cvtColor(eroded_image, greyErodedContourImage, CV_BGR2GRAY);
    greyErodedContourImage.convertTo(greyErodedContourImage, CV_32FC1);

    int thresh = 117; // Threshold for corner detection
    int blockSize = 2; // Size of neighborhood considered for corner detection
    int apertureSize = 3; // Aperture parameter for the Sobel operator
    double k = 0.04; // Harris detector free parameter

    cv::cornerHarris(greyErodedContourImage, harrisImg, blockSize, apertureSize, k);
    cv::normalize(harrisImg, harris_norm, 0, 255, cv::NORM_MINMAX, CV_32FC1);
    cv::convertScaleAbs(harris_norm, harris_norm_scaled);   

    std::vector<std::pair<int, cv::Point>> corners;
    int cornerId = 1;

    int imgCenterX = harris_norm.cols / 2;
    int imgCenterY = harris_norm.rows / 2;

    for (int i = 0; i < harris_norm.rows; i++) {
        for (int j = 0; j < harris_norm.cols; j++) {
            if ((int)harris_norm.at<float>(i, j) > thresh) {
                float worldX = (j - imgCenterX) * resolution;
                float worldY = (imgCenterY - i) * resolution;

                corners.push_back(std::make_pair(cornerId, cv::Point2f(worldX, worldY)));
                cv::circle(colorImage, cv::Point(j, i), 1, cv::Scalar(0, 255, 0), -1);
                cv::putText(colorImage, std::to_string(cornerId), cv::Point(j + 10, i + 10), cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(0, 255, 0), 1);
                cornerId++;
            }
        }
    }

    // Save the resulting image
    std::string outputImagePath = "/home/cocokayya18/Probabilistic-Robotics-Lab/src/kayyalakkakam/src/map/Altered_MapImages/Map_With_Corners.jpg";
    if (!cv::imwrite(outputImagePath, colorImage)) {
        std::cerr << "Failed to save the image to " << outputImagePath << std::endl;
    }

    filename = "/home/cocokayya18/Probabilistic-Robotics-Lab/src/kayyalakkakam/src/features/map_harris_features.csv";

    std::ofstream out_file2(filename);
    if (!out_file2) {
        std::cerr << "Failed to open file for writing.\n";
        return;
    }

    out_file2 << std::fixed;

    int x_threshold = 0; 
    int y_threshold = 0; 

    for (const auto& corner : corners) {
        // Check if the corner's x or y exceeds the threshold
        if (corner.second.x > x_threshold || corner.second.y != y_threshold) {
            continue; 
        }

        point cornerPoint;
        cornerPoint.x = corner.second.x;
        cornerPoint.y = corner.second.y;
        this->mapCornerFeatures.push_back(cornerPoint);

        // Log and write corners that pass the threshold check
        out_file2 << corner.first << "," << corner.second.x << "," << corner.second.y << "\n";
    }

    out_file2.close();

    for (const auto& coords : mapCornerFeatures) {
        std::cout << "X: " << coords.x << " Y: " << coords.y << std::endl;
        map_Corner_Cloud->push_back(pcl::PointXYZ(coords.x, coords.y, 0));

        if (map_Corner_Cloud->points.empty()) {
            ROS_ERROR("map_Corner_Cloud is empty after detectCornersInMap.");
            } 
        else {
            // ROS_INFO_STREAM("map_Corner_Cloud has " << map_Corner_Cloud->points.size() << " points.");
        }
    }
}


void EKF_Localization::detectCornersInLidar(sensor_msgs::LaserScan input)
{

    sensor_msgs::PointCloud cloud;
    std::vector<point> cartesianPoints;

    // Convert LiDAR data into the map frame
    if (!transformer.waitForTransform("map", "base_scan", input.header.stamp, ros::Duration(0.5))) {
        ROS_WARN("Waiting for the transform timed-out.");
        return;
    }

    try {
        projector_.transformLaserScanToPointCloud("map", input, cloud, transformer);
    } catch (tf::TransformException& ex) {
        ROS_ERROR("%s", ex.what());
        return;
    }

    for (const auto& p : cloud.points) {
        cartesianPoints.push_back({p.x, p.y});
    }

    // Detect corners
    this->cornersInLidar = detectCorners(cartesianPoints);
    // for (const point& corner : this->cornersInLidar) {
    //     ROS_INFO_STREAM("Corner found at: (" << corner.x << ", " << corner.y << ")");
    // }
};

std::vector<point> EKF_Localization::detectCorners(const std::vector<point>& points) {
    std::vector<point> corners;
    if (points.size() < 3) return corners; // Need at least 3 points to form two vectors

    for (size_t i = 1; i < points.size() - 1; ++i) {
        point prev = points[i - 1];
        point curr = points[i];
        point next = points[i + 1];

        point vector1 = {prev.x - curr.x, prev.y - curr.y};
        point vector2 = {next.x - curr.x, next.y - curr.y};

        float dotProduct = vector1.x * vector2.x + vector1.y * vector2.y;
        float magnitude1 = sqrt(vector1.x * vector1.x + vector1.y * vector1.y);
        float magnitude2 = sqrt(vector2.x * vector2.x + vector2.y * vector2.y);
        float angle = acos(dotProduct / (magnitude1 * magnitude2));

        if (fabs(angle - M_PI/2) < 0.01) // About 90 degrees, adjust the threshold as needed 
        { 
            corners.push_back(curr);
        }
    }
    return corners;
};


// void EKF_Localization::detectCirclesInMap()
// {
//     std::string map_path = "/home/cocokayya18/Probabilistic-Robotics-Lab/src/kayyalakkakam/src/map/turtlebot3_world_map.pgm";
//     cv::Mat map_image = cv::imread(map_path, cv::COLOR_BGR2GRAY);
//     cv::Mat gaussianImg;
//     cv::Mat treshImg;
//     cv::Mat cannyImg;
//     cv::Mat colorImage;
//     cv::Mat harrisImg;
//     cv::Mat harris_norm;
//     cv::Mat harris_norm_scaled;
//     cv::Mat greyErodedContourImage;
//     cv::Mat sharpened_image;
//     cv::Mat eroded_image;

//     if (map_image.empty()) {
//         std::cerr << "Failed to load image at path: " << map_path << std::endl;
//         return;
//     }

//     cv::cvtColor(map_image, colorImage, cv::COLOR_GRAY2BGR);

//     cv::imwrite("/home/cocokayya18/Probabilistic-Robotics-Lab/src/kayyalakkakam/src/map/Altered_MapImages/Normal_Picture.jpg", map_image);


//     cv::GaussianBlur(map_image, gaussianImg, cv::Size(9,9), 3, 3);
//     cv::imwrite("/home/cocokayya18/Probabilistic-Robotics-Lab/src/kayyalakkakam/src/map/Altered_MapImages/Gaussian_Picture.jpg", gaussianImg);

//     cv::threshold(gaussianImg, treshImg,0,255,cv::THRESH_BINARY+cv::THRESH_OTSU);
//     cv::imwrite("/home/cocokayya18/Probabilistic-Robotics-Lab/src/kayyalakkakam/src/map/Altered_MapImages/Treshholded_Picture.jpg", treshImg);

//     cv::Canny(treshImg, cannyImg, 50, 150);
//     cv::imwrite("/home/cocokayya18/Probabilistic-Robotics-Lab/src/kayyalakkakam/src/map/Altered_MapImages/Canny_Picture.jpg", cannyImg);

//     std::vector<cv::Vec3f> circles;
//     cv::HoughCircles(cannyImg, circles, cv::HOUGH_GRADIENT, 1, cannyImg.rows/32, 100, 12, 1, 7);  

//      for( size_t i = 0; i < circles.size(); i++ )
//     {
//         cv::Vec3i c = circles[i];
//         Circle detectedCircle;
//         cv::Point center = cv::Point(c[0], c[1]);
//         int radius = c[2];
        
//         circle( colorImage, center, 1, cv::Scalar(0,255,255), 0.1, cv::LINE_AA);
//         circle( colorImage, center, radius, cv::Scalar(0,255,255), 0.1, cv::LINE_AA);
        
//         detectedCircle.center = cv::Point(c[0], c[1]);
//         detectedCircle.radius = c[2];
//         detectedCirclesInMap.push_back(detectedCircle);
//     }

//     imwrite("/home/cocokayya18/Probabilistic-Robotics-Lab/src/kayyalakkakam/src/map/Altered_MapImages/Map_With_Circles.jpg", colorImage);

//     float sumX = 0, sumY = 0;
//     for (const auto& circle : detectedCirclesInMap) {
//         sumX += circle.center.x;
//         sumY += circle.center.y;
//     }
//     float centerX = sumX / detectedCirclesInMap.size();
//     float centerY = sumY / detectedCirclesInMap.size();
    
//     for ( int features = 0; features < detectedCirclesInMap.size(); features++)
//     {
//         WorldCoords world;
//         world.x = (detectedCirclesInMap[features].center.x - centerX) * resolution;
//         world.y = (detectedCirclesInMap[features].center.y - centerY) * resolution;
//         world.radius = detectedCirclesInMap[features].radius * resolution;
//         mapCircleFeatures.push_back(world);  
//         // ROS_INFO_STREAM("Map Feature: x=" << mapCircleFeatures[features].x << ", y=" << mapCircleFeatures[features].y << ", radius=" << mapCircleFeatures[features].radius);
//     }

//     std::string filename = "/home/cocokayya18/Probabilistic-Robotics-Lab/src/kayyalakkakam/src/features/map_ransac_features.csv";

//     std::ofstream out_file(filename);
//     if (!out_file) {
//         std::cerr << "Failed to open file for writing.\n";
//         return;
//     }

//     out_file << std::fixed;

//     for (const auto& coords : mapCircleFeatures) {
//         out_file << coords.x << ", " << coords.y << ", " << coords.radius << '\n';
//         map_Circle_Cloud->push_back(pcl::PointXYZ(coords.x, coords.y, 0));
//     }

//     out_file.close();
// }


// void EKF_Localization::detectCircleInLidar(sensor_msgs::LaserScan input)
// {
//     sensor_msgs::PointCloud2 cloud2;

//     // Convert Lidar to point cloud and into the map frame
//     if (!transformer.waitForTransform("map", "base_scan", input.header.stamp, ros::Duration(0.5))) {
//     ROS_WARN("Waiting for the transform timed-out.");
//     return;
//     }

//     try {
//     projector_.transformLaserScanToPointCloud("map", input, cloud2, transformer);
//     } 
//     catch (tf::TransformException &ex) {
//         ROS_WARN("TF exception:\n%s", ex.what());
//         return;
//     }

//     pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
//     pcl::fromROSMsg(cloud2, *cloud);

//     // Check if the cloud is empty
//     if (cloud->points.empty()) {
//         ROS_WARN("Received an empty point cloud.");
//         return;
//     }

//     // Prepare the model coefficients and inliers to store results
//     int circle_id = 0;
//     const int maxCircles = 1; // Limit the number of circles to detect
//     std::vector<Circle> newCircles;

//     while (circle_id < maxCircles && !cloud->points.empty()) {
//         pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
//         pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
//         pcl::SACSegmentation<pcl::PointXYZ> seg;
//         seg.setOptimizeCoefficients(true);
//         seg.setModelType(pcl::SACMODEL_CIRCLE2D);
//         seg.setMethodType(pcl::SAC_RANSAC);
//         seg.setMaxIterations(1000);
//         seg.setDistanceThreshold(0.02);
//         seg.setRadiusLimits(0.05, 0.2);

//         seg.setInputCloud(cloud);
//         seg.segment(*inliers, *coefficients);

//         if (inliers->indices.size() == 0) {
//             ROS_WARN("No additional circles found.");
//             break;
//         }

//         // Log the detected circle's parameters and add to lidarFeature
//         // ROS_INFO("Detected circle %d with center (%f, %f) and radius %f", circle_id, coefficients->values[0], coefficients->values[1], coefficients->values[2]);

//         Circle feature;
//         feature.center.x = coefficients->values[0];
//         feature.center.y = coefficients->values[1];
//         feature.radius = coefficients->values[2];
//         newCircles.push_back(feature);

//         // ROS_INFO("Cloud size before inlier removal: %zu", cloud->points.size());

//         pcl::ExtractIndices<pcl::PointXYZ> extract;
//         extract.setInputCloud(cloud);
//         extract.setIndices(inliers);
//         extract.setNegative(true);
//         extract.filter(*cloud);

//         // ROS_INFO("Cloud size after inlier removal: %zu", cloud->points.size());

//         circle_id++;

//         // Filter the Features, which are not in range of the real features
//         std::vector<Circle> filteredCircles;
//         for (const auto& newCircle : newCircles) {
//             if (newCircle.center.x > -1.2 && newCircle.center.x < 1.2 && newCircle.center.y > -1.2 && newCircle.center.y < 1.2) {
//                 // ROS_INFO_STREAM("NEW CIRCLE ADDED");
//                 filteredCircles.push_back(newCircle);
//             }
//             else {
//                 // ROS_INFO_STREAM("Filtered out circle at x " << newCircle.center.x << " and y " << newCircle.center.y <<  "with radius" << newCircle.radius << "\n");
//             }
//         }

//         newCircles = filteredCircles;

//         // Keep the newest Features in the vector
//         if (detectedCirclesInLidar.size() + newCircles.size() > maxRansacFeatureSize) {
//             // Remove the first N features to make room for the new ones
//             int featuresToRemove = detectedCirclesInLidar.size() + newCircles.size() - maxRansacFeatureSize;
//             detectedCirclesInLidar.erase(detectedCirclesInLidar.begin(), detectedCirclesInLidar.begin() + std::min(featuresToRemove, (int)detectedCirclesInLidar.size()));
//         }
//         detectedCirclesInLidar.insert(detectedCirclesInLidar.end(), newCircles.begin(), newCircles.end()); // Add new circles

//         // Save to csv file
//         if (!detectedCirclesInLidar.empty()) {
//             std::string filename = "/home/cocokayya18/Probabilistic-Robotics-Lab/src/kayyalakkakam/src/features/ransac_features.csv";
//             std::ofstream out_file(filename);
//             if (!out_file) {
//                 std::cerr << "Failed to open file for writing.\n";
//                 return;
//             }
//             out_file << std::fixed;
//             for (const auto& circle : detectedCirclesInLidar) {
//                 out_file << circle.center.x << ", " << circle.center.y << ", " << circle.radius << '\n';
//             }
//             out_file.close();
//             // ROS_INFO_STREAM("Ransac Features saved in world coordinates to " << filename << '\n');
//         }
//     }
// }


/// Feature Extraction ///