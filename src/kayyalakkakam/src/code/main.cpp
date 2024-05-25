#include "ros/ros.h"
#include "EKF_Localization.h"

int main(int argc, char** argv){

    ros::init(argc, argv, "main_node");                                                               
    ROS_INFO_STREAM("Main Node started");
    ros::NodeHandle nh("~");
    EKF_Localization slamer(nh);

    ros::Rate loop_rate(30);


    while (ros::ok()) {
        ros::spinOnce();
        loop_rate.sleep();
        slamer.run_EKF_Filter();
    }

    ROS_INFO_STREAM("CLOSING EVERYTHING");                                                           
    return 0;
};