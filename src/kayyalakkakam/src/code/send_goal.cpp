#include "ros/ros.h"
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/Twist.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <tf/tf.h>
#include <signal.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

// Function to move the robot to the specified goal
void moveToGoal(MoveBaseClient& client, double x_goal, double y_goal, double yaw_goal) {
    move_base_msgs::MoveBaseGoal goal;

    // Set up the frame parameters
    goal.target_pose.header.frame_id = "map";  // Change to your frame ID
    goal.target_pose.header.stamp = ros::Time::now();

    // Define position
    goal.target_pose.pose.position.x = x_goal;
    goal.target_pose.pose.position.y = y_goal;

    // Convert yaw to quaternion using tf library
    tf::Quaternion q = tf::createQuaternionFromYaw(yaw_goal);
    geometry_msgs::Quaternion q_msg;
    tf::quaternionTFToMsg(q, q_msg);
    goal.target_pose.pose.orientation = q_msg;

    // Send the goal to the action server
    client.sendGoal(goal);

    // Wait for the action server to complete the goal
    if (client.waitForResult()) {
        if (client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
            ROS_INFO("Successfully reached the waypoint.");
        else
            ROS_INFO("Failed to reach the waypoint.");
    } else {
        ROS_ERROR("Action server not available!");
        ros::shutdown();
    }
}

// Function to initialize the action client
MoveBaseClient initializeActionClient() {
    MoveBaseClient client("move_base", true);
    client.waitForServer();
    return client;
}

// Function to stop the robot
void stopRobot() {
    ros::NodeHandle n;
    ros::Publisher pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
    ros::Duration(1.0).sleep(); // Sleep to give the publisher time to connect

    geometry_msgs::Twist stop_msg; // Default constructor initializes to zero
    pub.publish(stop_msg);
    ROS_INFO("Robot stopped.");
}

// Function to stop ROS master and all nodes
void stopRosMaster() {
    ROS_INFO("Stopping ROS master and all nodes.");
    system("rosnode kill -a");
    system("killall -9 rosmaster");
    system("killall -9 roscore");
}

// Function to execute movement in a square pattern
void moveSquare(MoveBaseClient& client) {
    std::vector<std::tuple<double, double, double>> waypoints = {
        std::make_tuple(-1.5, 1.5, M_PI/2),
        std::make_tuple(-1.5, -1.5, M_PI),
        std::make_tuple(1.5, -1.5, -M_PI/2),
        std::make_tuple(1.5, 1.5, 0)
    };

    for (auto& waypoint : waypoints) {
        double x_goal, y_goal, yaw_goal;
        std::tie(x_goal, y_goal, yaw_goal) = waypoint;
        ROS_INFO("Moving to waypoint: x = %f, y = %f meters", x_goal, y_goal);
        moveToGoal(client, x_goal, y_goal, yaw_goal);
        ros::Duration(1.0).sleep(); // Short pause at each waypoint
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "move_base_square_cpp");
    MoveBaseClient client = initializeActionClient();

    try {
        moveSquare(client);
        stopRobot(); // Ensure the robot is stopped after completing the square
        stopRosMaster(); // Shutdown ROS master
    } catch (const ros::Exception& e) {
        ROS_ERROR("Navigation sequence interrupted: %s", e.what());
    }

    return 0;
}
