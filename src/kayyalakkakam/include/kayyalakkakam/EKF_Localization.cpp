#include "EKF_Localization.h"

EKF_Localization::EKF_Localization(ros::NodeHandle &N) : NH(N), robot(NH)
{
    ROS_INFO_STREAM("Initializing EKF NODE");
                                                                                    // Initialize the state vector and covariance matrix
    mu = Eigen::VectorXd(3);
    mu << 1.5, 1.5, 3.1415; 
    delta = Eigen::VectorXd(3);
    z_hat = Eigen::VectorXd(3);                                                        // Initiate State vector
    Sigma = Eigen::MatrixXd::Identity(3, 3);                                        // Initial covariance as Identity Matrix
    R = Eigen::MatrixXd::Identity(3, 3);                                            // Initial Process Noise Matrix as Identity Matrix
    Q = Eigen::MatrixXd::Identity(3,3);
    KalmanGain = Eigen::MatrixXd::Identity(3,3);
    Q << sensor_noise, 0, 0, 0, sensor_noise, 0, 0, 0, 1;
    g_function_jacobi = Eigen::MatrixXd::Identity(3, 3);                            // Initial Jacobian of the g_function
    h_function_jacobi = Eigen::MatrixXd::Identity(3, 3);                            // Initial Jacobian of the h_function
    map_cloud.reset(new pcl::PointCloud<pcl::PointXYZ>());                          // Initialize Space for map feature point cloud
    detectCirclesInMap();                                                           // Extract Features from Map
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

    robot.publishEKFpath(path);

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

    robot.publishOdometryPath(path);
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

void EKF_Localization::publishRansacFeatures()
{
    ROS_INFO_STREAM("Publishing Ransac Feature Marker Array");

    visualization_msgs::MarkerArray markerArray;

    for (size_t i = 0; i < detectedCirclesInLidar.size(); i++) {
        const auto& feature = detectedCirclesInLidar[i];
        
        visualization_msgs::Marker marker;
        marker.header.frame_id = "map";  // Ensure this matches your coordinate system in RViz
        marker.header.stamp = ros::Time::now();
        marker.ns = "circles";
        marker.id = i;
        marker.type = visualization_msgs::Marker::CYLINDER;
        marker.action = visualization_msgs::Marker::ADD;

        marker.pose.position.x = feature.center.x;
        marker.pose.position.y = feature.center.y;
        marker.pose.position.z = 0;  
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;

        marker.scale.x = feature.radius * 2;  
        marker.scale.y = feature.radius * 2;  
        marker.scale.z = 0.1;                 
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 1.0;  // Purple for visualization
        marker.color.a = 0.8;  

        marker.lifetime = ros::Duration();  

        markerArray.markers.push_back(marker);
    }

    robot.publishRansacFeatures(markerArray);
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
    for (size_t i = 0; i < detectedCirclesInLidar.size(); ++i) {
        if (Indices[i] != -1) { 
            auto detectedFeature = detectedCirclesInLidar[i];
            auto mapFeature = mapFeatures[Indices[i]];


            delta(0) = mapFeature.x - mu(0);
            delta(1) = mapFeature.y - mu(1);

            double q = delta.squaredNorm();

            z_hat << std::sqrt(q),
                     std::atan2(delta.y(), delta.x()) - mu(2),
                     mapFeature.radius;

            h_function_jacobi << delta.x() / std::sqrt(q), delta.y() / std::sqrt(q), 0,
                                 -delta.y() / q, delta.x() / q, -1,
                                 0, 0, 0;

            Eigen::MatrixXd S = h_function_jacobi * Sigma * h_function_jacobi.transpose() + Q;
            KalmanGain = Sigma * h_function_jacobi.transpose() * S.inverse();

            Eigen::Vector3d z_difference;
            z_difference << detectedFeature.center.x - z_hat(0),
                             detectedFeature.center.y - z_hat(1),
                             detectedFeature.radius - z_hat(2);

            mu = mu + KalmanGain * z_difference;
            Sigma = (Eigen::MatrixXd::Identity(mu.size(), mu.size()) - KalmanGain * h_function_jacobi) * Sigma;
        } 
        
        else {
            ROS_WARN_STREAM("No valid match for detected feature");
        }
    }
}

std::vector<int> EKF_Localization::landmarkMatching(const std::vector<Circle>& detectedFeatures)
{
    kdtree.setInputCloud(map_cloud);

    std::vector<int> indices(detectedFeatures.size(), -1);
        for (size_t i = 0; i < detectedFeatures.size(); ++i) {
            pcl::PointXYZ searchPoint(detectedFeatures[i].center.x, detectedFeatures[i].center.y, 0);
            std::vector<int> pointIdxNKNSearch(1);
            std::vector<float> pointNKNSquaredDistance(1);

            if (kdtree.nearestKSearch(searchPoint, 1, pointIdxNKNSearch, pointNKNSquaredDistance) > 0) {
                indices[i] = pointIdxNKNSearch[0];
            }
        }
        return indices;
}

void EKF_Localization::prediction_step()
{
    ROS_INFO_STREAM("PREDICTION RUNNING");
    this->odomMessage = this->robot.getOdom();

    mu = mu + g_function(mu, this->odomMessage);
    mu(2) = atan2(sin(mu(2)), cos(mu(2)));
    Sigma = updateSigma(mu, this->odomMessage);

    // this->publishEKFPath();
    // this->publishCovariance();
    // this->publishEKFPose();
    this->publishOdometryPath();

    // this->printMuAndSigma();
};

void EKF_Localization::correction_step()
{   
    ROS_INFO_STREAM("CORRECTION RUNNING");
    this->laserscanMessage = this->robot.getLaserscan();

    detectCircleInLidar(laserscanMessage);

    ROS_INFO("Detected %lu circles in LIDAR data.", detectedCirclesInLidar.size());

    std::vector<int> matchedIndices = landmarkMatching(detectedCirclesInLidar);

    // Process each matched feature
    h_Function(matchedIndices);
    publishRansacFeatures();
    publishEKFPath();
    publishCovariance();
    publishEKFPose();
    printMuAndSigma();

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

    float sumX = 0, sumY = 0;
    for (const auto& circle : detectedCirclesInMap) {
        sumX += circle.center.x;
        sumY += circle.center.y;
    }
    float centerX = sumX / detectedCirclesInMap.size();
    float centerY = sumY / detectedCirclesInMap.size();
    
    for ( int features = 0; features < detectedCirclesInMap.size(); features++)
    {
        WorldCoords world;
        world.x = (detectedCirclesInMap[features].center.x - centerX) * resolution;
        world.y = (detectedCirclesInMap[features].center.y - centerY) * resolution;
        world.radius = detectedCirclesInMap[features].radius * resolution;
        mapFeatures.push_back(world);   

        std::string filename = "/home/cocokayya18/Probabilistic-Robotics-Lab/src/kayyalakkakam/src/features/map_features.csv";

        std::ofstream out_file(filename);
        if (!out_file) {
            std::cerr << "Failed to open file for writing.\n";
            return;
        }

        out_file << std::fixed;

        for (const auto& coords : mapFeatures) {
            out_file << coords.x << ", " << coords.y << ", " << world.radius << '\n';
            map_cloud->push_back(pcl::PointXYZ(coords.x, coords.y, 0));
        }

        out_file.close();
        // ROS_INFO_STREAM("Map Features saved in world coordinates to " << filename << '\n');
    }
}

void EKF_Localization::detectCircleInLidar(sensor_msgs::LaserScan input)
{
    sensor_msgs::PointCloud2 cloud2;

    // Convert Lidar to point cloud and into the map frame
    if (!transformer.waitForTransform("map", "base_scan", input.header.stamp, ros::Duration(0.5))) {
    ROS_WARN("Waiting for the transform timed-out.");
    return;
    }

    try {
    projector_.transformLaserScanToPointCloud("map", input, cloud2, transformer);
    } 
    catch (tf::TransformException &ex) {
        ROS_WARN("TF exception:\n%s", ex.what());
        return;
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(cloud2, *cloud);

    // Check if the cloud is empty
    if (cloud->points.empty()) {
        ROS_WARN("Received an empty point cloud.");
        return;
    }

    // Prepare the model coefficients and inliers to store results
    int circle_id = 0;
    const int maxCircles = 1; // Limit the number of circles to detect
    std::vector<Circle> newCircles;

    while (circle_id < maxCircles && !cloud->points.empty()) {
        pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
        pcl::SACSegmentation<pcl::PointXYZ> seg;
        seg.setOptimizeCoefficients(true);
        seg.setModelType(pcl::SACMODEL_CIRCLE2D);
        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setMaxIterations(1000);
        seg.setDistanceThreshold(0.02);
        seg.setRadiusLimits(0.05, 0.2);

        seg.setInputCloud(cloud);
        seg.segment(*inliers, *coefficients);

        if (inliers->indices.size() == 0) {
            ROS_WARN("No additional circles found.");
            break;
        }

        // Log the detected circle's parameters and add to lidarFeature
        // ROS_INFO("Detected circle %d with center (%f, %f) and radius %f", circle_id, coefficients->values[0], coefficients->values[1], coefficients->values[2]);

        Circle feature;
        feature.center.x = coefficients->values[0];
        feature.center.y = coefficients->values[1];
        feature.radius = coefficients->values[2];
        newCircles.push_back(feature);

        // ROS_INFO("Cloud size before inlier removal: %zu", cloud->points.size());

        pcl::ExtractIndices<pcl::PointXYZ> extract;
        extract.setInputCloud(cloud);
        extract.setIndices(inliers);
        extract.setNegative(true);
        extract.filter(*cloud);

        // ROS_INFO("Cloud size after inlier removal: %zu", cloud->points.size());

        circle_id++;

        // Filter the Features, which are not in range of the real features
        std::vector<Circle> filteredCircles;
        for (const auto& circle : newCircles) {
            if (circle.center.x > -1.2 && circle.center.x < 1.2 && circle.center.y > -1.2 && circle.center.y < 1.2) {
                filteredCircles.push_back(circle);
            } else {
                // ROS_INFO_STREAM("Filtered out circle at x " << circle.center.x << " and y " << circle.center.y <<  "with radius" << circle.radius << "\n");
            }
        }
        newCircles = filteredCircles;

        // Keep the newest Features in the vector
        if (detectedCirclesInLidar.size() + newCircles.size() > maxRansacFeatureSize) {
            // Remove the first N features to make room for the new ones
            int featuresToRemove = detectedCirclesInLidar.size() + newCircles.size() - maxRansacFeatureSize;
            detectedCirclesInLidar.erase(detectedCirclesInLidar.begin(), detectedCirclesInLidar.begin() + std::min(featuresToRemove, (int)detectedCirclesInLidar.size()));
        }
        detectedCirclesInLidar.insert(detectedCirclesInLidar.end(), newCircles.begin(), newCircles.end()); // Add new circles

        // Save to csv file
        if (!detectedCirclesInLidar.empty()) {
            std::string filename = "/home/cocokayya18/Probabilistic-Robotics-Lab/src/kayyalakkakam/src/features/ransac_features.csv";
            std::ofstream out_file(filename);
            if (!out_file) {
                std::cerr << "Failed to open file for writing.\n";
                return;
            }
            out_file << std::fixed;
            for (const auto& circle : detectedCirclesInLidar) {
                out_file << circle.center.x << ", " << circle.center.y << ", " << circle.radius << '\n';
            }
            out_file.close();
            // ROS_INFO_STREAM("Ransac Features saved in world coordinates to " << filename << '\n');
        }
    }
}

/// Feature Extraction ///