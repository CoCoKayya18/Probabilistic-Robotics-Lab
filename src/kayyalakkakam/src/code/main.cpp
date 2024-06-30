#include "ros/ros.h"
#include "EKF_Localization.h"
#include "robot_class.h"

int main(int argc, char** argv){

    ros::init(argc, argv, "main_node");                                                               
    ROS_INFO_STREAM("Main Node started");
    ros::NodeHandle nh("~");
    // EKF_Localization localizer(nh);
    Ros_Subscriber_Publisher_Class robot(nh);

    ros::Rate loop_rate(30);


    while (ros::ok()) {
        ros::spinOnce();
        loop_rate.sleep();
    }

    ROS_INFO_STREAM("CLOSING EVERYTHING");                                                           
    return 0;
};