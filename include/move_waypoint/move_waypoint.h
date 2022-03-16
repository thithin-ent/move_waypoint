#ifndef MOVE_WAYPOINT
#define MOVE_WAYPOINT

#include <math.h>
#include <string>
#include <iostream>
#include <fstream>

#include "ros/ros.h"
#include <ros/package.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Twist.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <actionlib/server/simple_action_server.h>

typedef actionlib::SimpleActionServer<move_base_msgs::MoveBaseAction> server;
using namespace std;

class Move_waypoints
{
public:
    Move_waypoints();
    ~Move_waypoints();
    void action(const move_base_msgs::MoveBaseGoalConstPtr& goal);
    double turn2goal(const geometry_msgs::TransformStamped &transformStamped, const move_base_msgs::MoveBaseGoalConstPtr &goal, const double &yaw);
    double endturn(const geometry_msgs::TransformStamped &transformStamped, const move_base_msgs::MoveBaseGoalConstPtr &goal, const double &yaw);
private:

protected:
    ros::NodeHandle nh_;
    ros::Publisher cmd_vel_;
    ros::Publisher path_plan_;
    int state_;
    tf2_ros::Buffer tfBuffer_;
    server* as_;
};

#endif
