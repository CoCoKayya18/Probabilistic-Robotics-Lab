#include "ros/ros.h"
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/Twist.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <tf/tf.h>
#include <signal.h>
#include <math.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

struct Goal {
    double x;
    double y;
    double yaw;
};

void sendGoal(Goal currentGoal)
{
    //tell the action client that we want to spin a thread by default
    MoveBaseClient ac("move_base", true);

    // Convert given Yaw-Goal to Quaternions
    tf::Quaternion goalQuatz;
    goalQuatz.setRPY(0, 0, currentGoal.yaw);
    goalQuatz = goalQuatz.normalize();

    //wait for the action server to come up
    while(!ac.waitForServer(ros::Duration(5.0))){
        ROS_INFO("Waiting for the move_base action server to come up");
    }

    //Define the goal as a movebase message

    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();
    goal.target_pose.pose.position.x = currentGoal.x;
    goal.target_pose.pose.position.y = currentGoal.y;
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

    std::vector<Goal> goals = {{-1.5, 1.5, M_PI/2}, {-1.5, -1.5, -M_PI}, {1.5, -1.5, -M_PI/2}, {1.5, 1.5, 0}};
    
    for(const auto& goal : goals) {
        sendGoal(goal);
    };
}

// Base-Code from https://wiki.ros.org/navigation/Tutorials/SendingSimpleGoals  