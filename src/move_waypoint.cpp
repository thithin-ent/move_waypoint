#include <move_waypoint/move_waypoint.h>

Move_waypoints::Move_waypoints() : as_(NULL)
{
    as_ = new server(nh_, "move_base", boost::bind(&Move_waypoints::action, this, _1), false);
    as_->start();
    cmd_vel_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1);
    path_plan_ = nh_.advertise<nav_msgs::Path>("waypoint_path", 1);
    obcheck_sub_ = nh_.subscribe("obstacle_check", 1, &Move_waypoints::obcheck_callback, this);
}

Move_waypoints::~Move_waypoints()
{
}

void Move_waypoints::obcheck_callback(const std_msgs::Bool::ConstPtr &data)
{
    obcheck_ = data->data;
}

void Move_waypoints::action(const move_base_msgs::MoveBaseGoalConstPtr &waypoint_goal)
{
    tf2_ros::TransformListener tfListener(tfBuffer_);
    ros::Rate rate(20.0);
    geometry_msgs::TransformStamped transformStamped;
    geometry_msgs::PoseStamped goal;
    geometry_msgs::Twist cmd_vel;

    goal.header.stamp = ros::Time::now();
    goal.header.frame_id = "map";
    goal.pose = waypoint_goal->target_pose.pose;
    
    double speed = 0; 

    state_ = 0;
    while (nh_.ok())
    {
        if(as_->isPreemptRequested()){
            if(as_->isNewGoalAvailable()){
                move_base_msgs::MoveBaseGoal new_goal = *as_->acceptNewGoal();
                goal.header.stamp = ros::Time::now();
                goal.header.frame_id = "map";
                goal.pose = new_goal.target_pose.pose;        
            }
        }

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
        //cout << state_ << endl;
        if (obcheck_){
            cmd_vel.angular.z = 0.0;
            cmd_vel.linear.x = 0.0;  
        }
        else if (state_ == 0){
            cmd_vel.angular.z = turn2goal(transformStamped, goal, yaw);
            cmd_vel.linear.x = 0.0;
            speed = 0;
        }
        else if (state_ == 1){
            cmd_vel.angular.z = turn2goal(transformStamped, goal, yaw);
            cmd_vel.linear.x = 0.5 -  0.5*exp(-speed);
            speed = speed + 0.1*exp(-speed+0.1);
        }
        else if (state_ == 2){
            cmd_vel.angular.z = endturn(transformStamped, goal, yaw);
            cmd_vel.linear.x = 0.0;    
        }
        else if (state_ == 3){
            cmd_vel.angular.z = 0.0;
            cmd_vel.linear.x = 0.0;
            state_ = 0;
            //cout << " success " << endl;
            as_->setSucceeded(move_base_msgs::MoveBaseResult(), "Goal reached.");
            break;
        }
        else {
            cmd_vel.angular.z = 0.0;
            cmd_vel.linear.x = 0.0;
            state_ = 0;
            as_->setAborted(move_base_msgs::MoveBaseResult(), "Failed to Goal reached.");
            //cout << " fail " << endl;
            break;
        }
        cout << "cmd_vel.angular.z: "<< cmd_vel.angular.z << endl;
        cout << "cmd_vel.linear.x: "<< cmd_vel.linear.x << endl;
        cmd_vel_.publish(cmd_vel);
        planpub(goal, transformStamped);
        // const geometry_msgs::PoseStamped& current_position = pose;
        // move_base_msgs::MoveBaseFeedback feedback;
        // feedback.base_position = current_position;
        // as_->publishFeedback(feedback);

        ros::spinOnce();
        rate.sleep();
    }
}

double Move_waypoints::turn2goal(const geometry_msgs::TransformStamped &transformStamped, const geometry_msgs::PoseStamped &goal,const double &yaw)
{
    double temp1, temp2, goal_head;
    temp1 = goal.pose.position.x - transformStamped.transform.translation.x;
    temp2 = goal.pose.position.y - transformStamped.transform.translation.y;
    goal_head = (atan2(temp2, temp1) - yaw);


    if (goal_head < -M_PI)
    {
        goal_head = (M_PI*2 - goal_head);
    }
    else if(goal_head > M_PI)
    {
        goal_head = (goal_head - M_PI*2); 
    }

    if (sqrt( pow(temp1,2) + pow(temp2,2) ) < 0.2 && state_ == 1) state_++; 
    else if (abs(goal_head) < 0.05 && state_ == 0) state_++;
    
    
    if (goal_head > 0){
        goal_head = (atan(2*goal_head-3.14)+2)/6 - 0.1*state_;
    }
    else {
        goal_head = -(atan(2*(-goal_head)-3.14)+2)/6 + 0.1*state_;
    }
    //cout << "cout << goal_head << endl: "<<goal_head << endl;
    //cout << "atan2(temp2, temp1): "<<atan2(temp2, temp1) << endl;
    //cout << "yaw: "<<yaw << endl;
    
    return goal_head;
}

double Move_waypoints::endturn(const geometry_msgs::TransformStamped &transformStamped, const geometry_msgs::PoseStamped &goal, const double &yaw)
{
    double goal_head;
    tf2::Quaternion q(goal.pose.orientation.x, goal.pose.orientation.y, goal.pose.orientation.z, goal.pose.orientation.w);
    double temp1, temp2, goal_yaw;
    tf2::Matrix3x3(q).getRPY(temp1, temp2, goal_yaw);
    goal_head = goal_yaw - yaw;

    if (goal_head < -M_PI)
    {
        goal_head = (M_PI*2 - goal_head);
    }
    else if(goal_head > M_PI)
    {
        goal_head = (goal_head - M_PI*2); 
    }

    if (abs(goal_head) < 0.05 && state_ == 2) state_++;
    
    if (goal_head > 0){
        goal_head = (atan(2*goal_head-3.14)+2)/6;
    }
    else {
        goal_head = -(atan(2*(-goal_head)-3.14)+2)/6;
    }

    return goal_head;
}

void Move_waypoints::planpub(const geometry_msgs::PoseStamped &goal, const geometry_msgs::TransformStamped &transformStamped){
    nav_msgs::Path path;
    geometry_msgs::PoseStamped pose;
    path.header.stamp = ros::Time::now();
    path.header.frame_id = "map";
    pose.header.stamp = ros::Time::now();
    pose.header.frame_id = "map" ;
    pose.pose.position.x = transformStamped.transform.translation.x;
    pose.pose.position.y = transformStamped.transform.translation.y;
    pose.pose.position.z = transformStamped.transform.translation.z;
    pose.pose.orientation = transformStamped.transform.rotation;
    path.poses.push_back(goal);
    path.poses.push_back(pose);
    path_plan_.publish(path);
}

