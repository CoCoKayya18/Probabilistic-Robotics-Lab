#include "robot_class.h"
#include "send_goal.h"


int main(int argc, char** argv){

    ros::init(argc, argv, "simple_navigation_goals");

    std::vector<GoalSender::Goal> goals = {{-1.5, 1.5, M_PI/2}, {-1.5, -1.5, -M_PI}, {1.5, -1.5, -M_PI/2}, {1.5, 1.5, 0}};
    GoalSender theGoaler(goals);

    ros::init(argc, argv, "my_ros_node");                                                               
    ros::NodeHandle nh("~");                                                                            
    MyClass robot(nh);                                                                                  
    // ros::Timer timer = nh.createTimer(ros::Duration(robot.pub_rate), &MyClass::timer_callback, &robot);
    ros::spin();                                                                                        
    return 0;
}
