#include "robot_class.h"

MyClass::MyClass(ros::NodeHandle &N):NH(N){
    ROS_INFO_STREAM("Init My ROS Package");
    pub = NH.advertise< std_msgs::String >("egal", 1 );
    check_params(NH);
}

MyClass::~MyClass( void ){}

void MyClass::odom_callback( const nav_msgs::Odometry::ConstPtr& msg ) {
    ROS_INFO_STREAM( "Got Odom Message" );
    ROS_INFO_STREAM(msg);
 }

 void MyClass::laserscan_callback( const sensor_msgs::LaserScan::ConstPtr& msg ) {
    ROS_INFO_STREAM( "Got LaserScan Message" );
    ROS_INFO_STREAM(msg);
 }

void MyClass::timer_callback(const ros::TimerEvent& event){
    ROS_INFO_STREAM( "Timer Callback" );
    my_message.data = "mymessage";
    pub.publish( my_message );
 }

void MyClass::check_params(ros::NodeHandle &N){
    ROS_INFO_STREAM( "Check Rate" );
    N.getParam("node_rate" , pub_rate);
    if(pub_rate > 10 || pub_rate < 1){
        pub_rate = 1;
    } 
    std::cout << "Publishing rate:" << pub_rate << std::endl;
 }


int main(int argc, char** argv){

    ros::init(argc, argv, "my_ros_node");                                                               
    ROS_INFO_STREAM("Subscriber Node started");
    ros::NodeHandle nh("~");
                                                                         
    MyClass robot(nh);    
    ROS_INFO_STREAM("Subscriber Node ended");                                                                        
    // ros::Timer timer = nh.createTimer(ros::Duration(robot.pub_rate), &MyClass::timer_callback, &robot);
    ros::spin();                                                                                        
    return 0;
};