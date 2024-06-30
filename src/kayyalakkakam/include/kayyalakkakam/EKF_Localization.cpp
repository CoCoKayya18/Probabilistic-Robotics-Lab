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

            Eigen::MatrixXd S = h_function_jacobi * Sigma * h_function_jacobi.transpose() + Q;
            KalmanGain = Sigma * h_function_jacobi.transpose() * S.inverse();

            z_difference = z_polar - z_hat;

            Kalman_Sensor_Sum_for_Mu = Kalman_Sensor_Sum_for_Mu + KalmanGain * z_difference;
            Kalman_H_Matrix_Sigma_Sum_For_Sigma = Kalman_H_Matrix_Sigma_Sum_For_Sigma + (Eigen::MatrixXd::Identity(mu.size(), mu.size()) - KalmanGain * h_function_jacobi);

        }   
    }

    mu = mu + Kalman_Sensor_Sum_for_Mu;
    Sigma = Kalman_H_Matrix_Sigma_Sum_For_Sigma * Sigma; 

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
            }
        }

        return indices;
};

void EKF_Localization::prediction_step()
{
    this->odomMessage = this->robot.getOdom();

    mu = mu + g_function(mu, this->odomMessage);
    mu(2) = atan2(sin(mu(2)), cos(mu(2)));
    Sigma = updateSigma(mu, this->odomMessage);
};

void EKF_Localization::correction_step()
{   
    this->laserscanMessage = this->robot.getLaserscan();

    detectCornersInLidar(laserscanMessage);

    std::vector<int> matchedIndices = landmarkMatching(this->cornersInLidar);

    // Process each matched feature
    h_Function(matchedIndices);
    publishCornerFeatures();
    publishEKFPath();
    publishCovariance();
    publishEKFPose();
    publishOdometryPath();
};

void EKF_Localization::run_EKF_Filter()
{
    this->prediction_step();
    this->correction_step();
};

/// Feature Extraction ///


void EKF_Localization::detectCornersInMap()
{
    std::string map_path = "../Probabilistic-Robotics-Lab/src/kayyalakkakam/src/map/turtlebot3_world_map.pgm";
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

    cv::imwrite("../Probabilistic-Robotics-Lab/src/kayyalakkakam/src/map/Altered_MapImages/Normal_Picture.jpg", map_image);


    cv::GaussianBlur(map_image, gaussianImg, cv::Size(9,9), 3, 3);
    cv::imwrite("../Probabilistic-Robotics-Lab/src/kayyalakkakam/src/map/Altered_MapImages/Gaussian_Picture.jpg", gaussianImg);

    cv::threshold(gaussianImg, treshImg,0,255,cv::THRESH_BINARY+cv::THRESH_OTSU);
    cv::imwrite("../Probabilistic-Robotics-Lab/src/kayyalakkakam/src/map/Altered_MapImages/Treshholded_Picture.jpg", treshImg);

    cv::Canny(treshImg, cannyImg, 50, 150);
    cv::imwrite("../Probabilistic-Robotics-Lab/src/kayyalakkakam/src/map/Altered_MapImages/Canny_Picture.jpg", cannyImg);

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
    cv::imwrite("../Probabilistic-Robotics-Lab/src/kayyalakkakam/src/map/Altered_MapImages/Contour_Picture.jpg", contourImage);

    cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
    cv::erode(contourImage, eroded_image, element);

    cv::imwrite("../Probabilistic-Robotics-Lab/src/kayyalakkakam/src/map/Altered_MapImages/Erored_Picture.jpg", eroded_image);

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
    std::string outputImagePath = "../Probabilistic-Robotics-Lab/src/kayyalakkakam/src/map/Altered_MapImages/Map_With_Corners.jpg";
    if (!cv::imwrite(outputImagePath, colorImage)) {
        std::cerr << "Failed to save the image to " << outputImagePath << std::endl;
    }

    filename = "../Probabilistic-Robotics-Lab/src/kayyalakkakam/src/features/map_harris_features.csv";

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
        // std::cout << "X: " << coords.x << " Y: " << coords.y << std::endl;
        map_Corner_Cloud->push_back(pcl::PointXYZ(coords.x, coords.y, 0));

        if (map_Corner_Cloud->points.empty()) {
            ROS_ERROR("map_Corner_Cloud is empty after detectCornersInMap.");
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

/// Feature Extraction ///