#ifndef OBSTACLE_CHECK
#define OBSTACLE_CHECK

#include <math.h>
#include <string>
#include <iostream>
#include <fstream>
#include <eigen3/Eigen/Dense>

#include "ros/ros.h"
#include <ros/package.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/PointCloud.h>
#include <std_msgs/Bool.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/utils.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <actionlib/server/simple_action_server.h>

using namespace std;
using namespace Eigen;

class Obstacle_check
{
public:
    Obstacle_check();
    ~Obstacle_check();
    void scan_callback(const sensor_msgs::LaserScanConstPtr &data);
    void basescan_make(const vector<Vector3f> &scan_poses);
    bool obstacle_check(const vector<Vector3f> &scan_poses);

private:
protected:
    ros::NodeHandle nh_;
    ros::Subscriber scan_sub_;
    double radius_param_;
    double radian_param_;
    Matrix3f transform_;
    ros::Publisher basescan_pub_;
    ros::Publisher obCheck_pub_;

};

#endif
