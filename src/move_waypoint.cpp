#include <move_waypoint/move_waypoint.h>

Move_waypoints::Move_waypoints() : as_(NULL)
{
    as_ = new server(nh_, "move_base", boost::bind(&Move_waypoints::action, this, _1), false);
    as_->start();
    cmd_vel_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1);
    cmd_vel_ = nh_.advertise<nav_msgs::Path>("waypoint_path", 1);
}

Move_waypoints::~Move_waypoints()
{
}

void Move_waypoints::action(const move_base_msgs::MoveBaseGoalConstPtr &goal)
{
    tf2_ros::TransformListener tfListener(tfBuffer_);
    ros::Rate rate(20.0);
    geometry_msgs::TransformStamped transformStamped;
    geometry_msgs::PoseStamped pose;
    geometry_msgs::Twist cmd_vel;
    
    nav_msgs::Path path;
    path.header = goal->target_pose.header;
    pose.header = goal->target_pose.header;
    pose.pose = goal->target_pose.pose;
    path.poses.push_back(pose);
    path.poses.push_back(pose);
    
    state_ = 0;
    while (nh_.ok())
    {
        try
        {
            transformStamped = tfBuffer_.lookupTransform("map", "base_footprint", ros::Time(0));
        }
        catch (tf2::TransformException ex)
        {
            ROS_ERROR("%s", ex.what());
            ros::Duration(0.1).sleep();
        }

        tf2::Quaternion q(transformStamped.transform.rotation.x, transformStamped.transform.rotation.y, transformStamped.transform.rotation.z, transformStamped.transform.rotation.w);
        double temp1, temp2, yaw;
        tf2::Matrix3x3(q).getRPY(temp1, temp2, yaw);
        if (state_ == 0){
            cmd_vel.angular.z = turn2goal(transformStamped, goal, yaw);
            cmd_vel.linear.x = 0.0;      
        }
        else if (state_ == 1){
            cmd_vel.angular.z = turn2goal(transformStamped, goal, yaw);
            cmd_vel.linear.x = 0.2;    
        }
        else if (state_ == 2){
            cmd_vel.angular.z = endturn(transformStamped, goal, yaw);
            cmd_vel.linear.x = 0.0;    
        }
        else if (state_ == 3){
            cmd_vel.angular.z = 0.0;
            cmd_vel.linear.x = 0.0;
            state_ = 0;
            as_->setSucceeded(move_base_msgs::MoveBaseResult(), "Goal reached.");
            break;
        }
        else {
            cmd_vel.angular.z = 0.0;
            cmd_vel.linear.x = 0.0;
            state_ = 0;
            as_->setAborted(move_base_msgs::MoveBaseResult(), "Failed to Goal reached.");
            break;
        }

        cmd_vel_.publish(cmd_vel);
        
        pose.header = goal->target_pose.header;
        pose.pose.position.x = transformStamped.transform.translation.x;
        pose.pose.position.y = transformStamped.transform.translation.y;
        pose.pose.position.z = transformStamped.transform.translation.z;
        pose.pose.orientation = transformStamped.transform.rotation;
        
        path.poses.emplace(path.poses.begin() + 1,pose);




        ros::spinOnce();
        rate.sleep();
    }
}

double Move_waypoints::turn2goal(const geometry_msgs::TransformStamped &transformStamped, const move_base_msgs::MoveBaseGoalConstPtr &goal,const double &yaw)
{
    double temp1, temp2, goal_head;
    temp1 = goal->target_pose.pose.position.x - transformStamped.transform.translation.x;
    temp2 = goal->target_pose.pose.position.y - transformStamped.transform.translation.y;
    goal_head = (atan2(temp2, temp1) - yaw)*0.3;
    if (sqrt( pow(temp1,2) + pow(temp2,2) ) < 0.2 && state_ == 1) state_++; 
    else if (goal_head < 0.05 && state_ == 0) state_++;
    else if (goal_head < 0.3 && state_ == 0) goal_head = 0.3;
    

    return goal_head;
}

double Move_waypoints::endturn(const geometry_msgs::TransformStamped &transformStamped, const move_base_msgs::MoveBaseGoalConstPtr &goal, const double &yaw)
{
    double goal_head;
    tf2::Quaternion q(goal->target_pose.pose.orientation.x, goal->target_pose.pose.orientation.y, goal->target_pose.pose.orientation.z, goal->target_pose.pose.orientation.w);
    double temp1, temp2, goal_yaw;
    tf2::Matrix3x3(q).getRPY(temp1, temp2, goal_yaw);
    goal_head = goal_yaw - yaw;
    if (goal_head < 0.05 && state_ == 2) state_++;
    else if (goal_head < 0.3 && state_ == 2) goal_head = 0.3;

    // cout << " value : " << goal->target_pose.pose.orientation.x << " value : " <<
    // goal->target_pose.pose.orientation.y << " value : " <<
    // goal->target_pose.pose.orientation.z << " value : " <<
    // goal->target_pose.pose.orientation.w << endl;

    // cout << "goal_yaw : " << goal_yaw << endl;
    // cout << "state : " << state_ << endl;
    // cout << "goal_head : " << goal_head << endl;

    return goal_head;
}