#include "ros/ros.h"

int main(int argc, char** argv){

    // ros::init(argc, argv, "my_ros_node");                                                               
    // ROS_INFO_STREAM("Node started");
    // ros::NodeHandle nh("~");

    // // std::vector<GoalSender::Goal> goals = {{-1.5, 1.5, M_PI/2}, {-1.5, -1.5, -M_PI}, {1.5, -1.5, -M_PI/2}, {1.5, 1.5, 0}};
    // // GoalSender theGoaler(goals);
    // // theGoaler.sendGoal();
                                                                         
    // MyClass robot(nh);    
    // ROS_INFO_STREAM("Node ended");                                                                        
    // // ros::Timer timer = nh.createTimer(ros::Duration(robot.pub_rate), &MyClass::timer_callback, &robot);
    ros::spin();                                                                                        
    return 0;
};