#ifndef ROBOT_CLASS_H_
#define ROBOT_CLASS_H_

#include "ros/ros.h"
#include "std_msgs/String.h"
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include "eigen3/Eigen/Dense"

class Ros_Subscriber_Publisher_Class{
    public:
        Ros_Subscriber_Publisher_Class(ros::NodeHandle &N);
        ~Ros_Subscriber_Publisher_Class( void );

        void odom_callback( const nav_msgs::Odometry::ConstPtr& msg);
        void laserscan_callback( const sensor_msgs::LaserScan::ConstPtr& msg);
        void map_callback ( const  nav_msgs::OccupancyGrid::ConstPtr& msg);

        // void timer_callback(const ros::TimerEvent& event);
        void check_params(ros::NodeHandle &N);

        nav_msgs::Odometry getOdom();
        sensor_msgs::LaserScan getLaserscan();
        nav_msgs::OccupancyGrid getMap();

        void publishEKFpath(nav_msgs::Path ekf_path);
        void publishCovariance(visualization_msgs::Marker marker);
        void publishOdometryPath(nav_msgs::Path odometry_path);
        void publishEKFPose(geometry_msgs::PoseStamped ekf_pose);
        void publishRansacFeatures(visualization_msgs::MarkerArray markerArray);
        

        int pub_rate;

    private:
        ros::NodeHandle NH;
        ros::Subscriber odomSub = NH.subscribe("/odom", 1, &Ros_Subscriber_Publisher_Class::odom_callback, this);
        ros::Subscriber laserSub = NH.subscribe("/scan", 1, &Ros_Subscriber_Publisher_Class::laserscan_callback, this);
        ros::Subscriber mapSub = NH.subscribe("/map", 1, &Ros_Subscriber_Publisher_Class::map_callback, this);
        std_msgs::String my_message;
        nav_msgs::Odometry myOdomMsg;
        sensor_msgs::LaserScan myLaserscanMsg;
        nav_msgs::OccupancyGrid myMapMsg;
        Eigen::Matrix<double, 3, 3> A;
        
        ros::Publisher pub;
        ros::Publisher EKF_path_pub;
        ros::Publisher covariance_marker_pub;
        ros::Publisher Odometry_path_pub;
        ros::Publisher EKF_Pose_Publisher;
        ros::Publisher Ransac_Features_Publisher;

};

#endif

/// Base-Code von Lucas Muster
