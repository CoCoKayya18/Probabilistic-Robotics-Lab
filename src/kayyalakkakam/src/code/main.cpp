#include "ros/ros.h"
#include "EKF_Localization.h"

int main(int argc, char** argv){

    ros::init(argc, argv, "my_ros_node");                                                               
    ROS_INFO_STREAM("Main Node started");
    ros::NodeHandle nh("~");
    EKF_Localization slamer(nh);

    ros::Rate loop_rate(10);

    while (ros::ok()) {
        slamer.run_EKF_Filter();
        ros::spinOnce();
        loop_rate.sleep();
    }

    ROS_INFO_STREAM("CLOSING EVERYTHING");                                                           
    return 0;
};