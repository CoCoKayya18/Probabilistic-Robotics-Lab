#include "robot_class.h"

Ros_Subscriber_Publisher_Class::Ros_Subscriber_Publisher_Class(ros::NodeHandle &N):NH(N){
    ROS_INFO_STREAM("Init My ROS Package");
    EKF_path_pub = NH.advertise< nav_msgs::Path >("EKF_path", 50, true);
    covariance_marker_pub = NH.advertise<visualization_msgs::Marker>("visualization_marker", 10);
    Odometry_path_pub = NH.advertise<nav_msgs::Path>("Odometry_path", 10, true);
    EKF_Pose_Publisher = NH.advertise<geometry_msgs::PoseStamped>("EKF_pose", 50, true);
    Corner_Features_Publisher = NH.advertise<visualization_msgs::Marker>("corner_features", 10);
    check_params(NH);
}

Ros_Subscriber_Publisher_Class::~Ros_Subscriber_Publisher_Class( void ){}

void Ros_Subscriber_Publisher_Class::odom_callback( const nav_msgs::Odometry::ConstPtr& msg ) {
    this->myOdomMsg = *msg;
    localizer.prediction_step(myOdomMsg);
    publishEKFPose();
    publishEKFpath();
    publishOdometryPath();
    publishCovariance();
    publishCornerFeatures();
 }

 void Ros_Subscriber_Publisher_Class::laserscan_callback( const sensor_msgs::LaserScan::ConstPtr& msg ) {
    this->myLaserscanMsg = *msg;
    localizer.correction_step(myLaserscanMsg);
    publishEKFPose();
    publishEKFpath();
    publishOdometryPath();
    publishCovariance();
    publishCornerFeatures();
 }

 void Ros_Subscriber_Publisher_Class::map_callback ( const  nav_msgs::OccupancyGrid::ConstPtr& msg){
    this->myMapMsg = *msg;
 }

void Ros_Subscriber_Publisher_Class::check_params(ros::NodeHandle &N){
    ROS_INFO_STREAM( "Check Rate" );
    N.getParam("node_rate" , pub_rate);
    if(pub_rate > 10 || pub_rate < 1){
        pub_rate = 1;
    } 
    std::cout << "Publishing rate:" << pub_rate << std::endl;
 }

nav_msgs::Odometry Ros_Subscriber_Publisher_Class::getOdom()
{
    return this->myOdomMsg;
}

sensor_msgs::LaserScan Ros_Subscriber_Publisher_Class::getLaserscan()
{
    return this->myLaserscanMsg;
}

nav_msgs::OccupancyGrid Ros_Subscriber_Publisher_Class::getMap()
{
    return this->myMapMsg;
}

void Ros_Subscriber_Publisher_Class::publishEKFpath()
{
    EKF_path_pub.publish(localizer.publishEKFPath());
}

void Ros_Subscriber_Publisher_Class::publishCovariance()
{
    covariance_marker_pub.publish(localizer.publishCovariance());
}

void Ros_Subscriber_Publisher_Class::publishOdometryPath()
{
    Odometry_path_pub.publish(localizer.publishOdometryPath());
}


void Ros_Subscriber_Publisher_Class::publishEKFPose()
{
    EKF_Pose_Publisher.publish(localizer.publishEKFPose());
}

void Ros_Subscriber_Publisher_Class::publishCornerFeatures()
{
    Corner_Features_Publisher.publish(localizer.publishCornerFeatures());
}