#include "send_goal.h"

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

struct GoalSender::Goal {
    double x;
    double y;
    double yaw;
};

void GoalSender::sendGoal()
{
    for(const auto& currentGoal : this->listOfGoals) {
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
    };
};


// Base-Code from https://wiki.ros.org/navigation/Tutorials/SendingSimpleGoals  