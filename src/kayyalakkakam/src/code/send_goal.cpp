#include "ros/ros.h"
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/Twist.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <tf/tf.h>
#include <signal.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

void sendGoal(double x_goal, double y_goal, double yaw_goal)
{
    //tell the action client that we want to spin a thread by default
    MoveBaseClient ac("move_base", true);

    // Convert given Yaw-Goal to Quaternions
    tf::Quaternion goalQuatz;
    goalQuatz.setRPY(0, 0, yaw_goal);
    goalQuatz = goalQuatz.normalize();

    //wait for the action server to come up
    while(!ac.waitForServer(ros::Duration(5.0))){
        ROS_INFO("Waiting for the move_base action server to come up");
    }

    //Define the goal as a movebase message

    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();
    goal.target_pose.pose.position.x = x_goal;
    goal.target_pose.pose.position.y = y_goal;
    goal.target_pose.pose.orientation.w = goalQuatz.getW();
    goal.target_pose.pose.orientation.x = goalQuatz.getX();
    goal.target_pose.pose.orientation.y = goalQuatz.getY();
    goal.target_pose.pose.orientation.z = goalQuatz.getZ();

    // Send goal
    ac.sendGoal(goal);
    ac.waitForResult();

    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
        ROS_INFO("robot reached its goal");
    else
        ROS_INFO("Robot failed");
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "simple_navigation_goals");

    // Define 4 goals
    std::tuple<double, double, double> Goal1 = {0, 3, 3.1415};
    std::tuple<double, double, double> Goal2 = {0, 3, 3.1415};
    std::tuple<double, double, double> Goal3 = {0, 3, 3.1415};
    std::tuple<double, double, double> Goal4 = {0, 3, 3.1415};
    
    for (int i=0; i++; i<4){
        sendGoal(i,0,1);
    } 
}

// Base-Code from https://wiki.ros.org/navigation/Tutorials/SendingSimpleGoals  