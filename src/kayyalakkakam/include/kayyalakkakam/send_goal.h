#include "ros/ros.h"
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/Twist.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <tf/tf.h>
#include <signal.h>
#include <math.h>

class GoalSender
{
    public:
        
        struct Goal {
            double x;
            double y;
            double yaw;

            Goal(double x, double y, double yaw) : x(x), y(y), yaw(yaw) {};
        };

        GoalSender(std::vector<Goal> goals):listOfGoals(goals){};
        ~GoalSender(){};
        void sendGoal();

    
    private:
        std::vector<Goal> listOfGoals;
};