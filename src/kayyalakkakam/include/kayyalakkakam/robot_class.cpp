#include "robot_class.h"

Ros_Subscriber_Publisher_Class::Ros_Subscriber_Publisher_Class(ros::NodeHandle &N):NH(N){
    ROS_INFO_STREAM("Init My ROS Package");
    pub = NH.advertise< std_msgs::String >("egal", 1 );
    check_params(NH);
}

Ros_Subscriber_Publisher_Class::~Ros_Subscriber_Publisher_Class( void ){}

void Ros_Subscriber_Publisher_Class::odom_callback( const nav_msgs::Odometry::ConstPtr& msg ) {
    // ROS_INFO_STREAM( "Got Odom Message" );
    this->myOdomMsg = *msg;
    // ROS_INFO_STREAM(this->myOdomMsg);
 }

 void Ros_Subscriber_Publisher_Class::laserscan_callback( const sensor_msgs::LaserScan::ConstPtr& msg ) {
    // ROS_INFO_STREAM( "Got LaserScan Message" );
    this->myLaserscanMsg = *msg;
    // ROS_INFO_STREAM(this->myLaserscanMsg);
 }

 void Ros_Subscriber_Publisher_Class::map_callback ( const  nav_msgs::OccupancyGrid::ConstPtr& msg){
    this->myMapMsg = *msg;
 }

void Ros_Subscriber_Publisher_Class::timer_callback(const ros::TimerEvent& event){
    ROS_INFO_STREAM( "Timer Callback" );
    my_message.data = "mymessage";
    pub.publish( my_message );
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