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
#include <std_msgs/Bool.h>
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
    double turn2goal(const geometry_msgs::TransformStamped &transformStamped, const geometry_msgs::PoseStamped &goal, const double &yaw);
    double endturn(const geometry_msgs::TransformStamped &transformStamped, const geometry_msgs::PoseStamped &goal, const double &yaw);
    void planpub(const geometry_msgs::PoseStamped &goal, const geometry_msgs::TransformStamped &transformStamped);
    void obcheck_callback(const std_msgs::Bool::ConstPtr &data);
private:

protected:
    move_base_msgs::MoveBaseResult result_;
    ros::NodeHandle nh_;
    ros::Publisher cmd_vel_;
    ros::Publisher path_plan_;
    ros::Subscriber obcheck_sub_;
    int state_;
    bool obcheck_;
    tf2_ros::Buffer tfBuffer_;
    server* as_;
};

#endif
