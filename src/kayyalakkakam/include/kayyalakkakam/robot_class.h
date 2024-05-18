#ifndef ROBOT_CLASS_H_
#define ROBOT_CLASS_H_

#include "ros/ros.h"
#include "std_msgs/String.h"
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include "eigen3/Eigen/Dense"

class Ros_Subscriber_Publisher_Class{
    public:
        Ros_Subscriber_Publisher_Class(ros::NodeHandle &N);
        ~Ros_Subscriber_Publisher_Class( void );
        void odom_callback( const nav_msgs::Odometry::ConstPtr& msg);
        void laserscan_callback( const sensor_msgs::LaserScan::ConstPtr& msg);
        void timer_callback(const ros::TimerEvent& event);
        void check_params(ros::NodeHandle &N);
        nav_msgs::Odometry getOdom();
        sensor_msgs::LaserScan getLaserscan();
        int pub_rate;

    private:
        ros::NodeHandle NH;
        ros::Publisher pub;
        ros::Subscriber odomSub = NH.subscribe("/odom", 1, &Ros_Subscriber_Publisher_Class::odom_callback, this);
        ros::Subscriber laserSub = NH.subscribe("/scan", 1, &Ros_Subscriber_Publisher_Class::laserscan_callback, this);
        std_msgs::String my_message;
        nav_msgs::Odometry myOdomMsg;
        sensor_msgs::LaserScan myLaserscanMsg;
        Eigen::Matrix<double, 3, 3> A;
};

#endif

/// Base-Code von Lucas Muster
