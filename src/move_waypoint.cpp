#include <move_waypoint/move_waypoint.h>

Move_waypoints::Move_waypoints() : as_(NULL)
{
    as_ = new server(nh_, "move_base", boost::bind(&Move_waypoints::action, this, _1), false);
    as_->start();
    cmd_vel_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1);
}

Move_waypoints::~Move_waypoints()
{
}

void Move_waypoints::action(const move_base_msgs::MoveBaseGoalConstPtr &goal)
{
    tf2_ros::TransformListener tfListener(tfBuffer_);
    ros::Rate rate(20.0);
    geometry_msgs::TransformStamped transformStamped;
    geometry_msgs::Twist cmd_vel;
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

        cmd_vel.angular.z = turn2goal(transformStamped, goal, yaw);
        cmd_vel.angular.x = 0;      
        //move2goal(transformStamped, goal, cmd_vel);
        //endturn();

        cmd_vel_.publish(cmd_vel);

        ros::spinOnce();
        rate.sleep();
    }
}

double Move_waypoints::turn2goal(const geometry_msgs::TransformStamped &transformStamped, const move_base_msgs::MoveBaseGoalConstPtr &goal,const double &yaw)
{
    double temp1, temp2, goal_head;
    temp1 = goal->target_pose.pose.position.x - transformStamped.transform.translation.x;
    temp2 = goal->target_pose.pose.position.y - transformStamped.transform.translation.y;
    goal_head = (yaw - atan2(temp2, temp1))*0.3;

    return goal_head;
}

double Move_waypoints::endturn(const geometry_msgs::TransformStamped &transformStamped, const move_base_msgs::MoveBaseGoalConstPtr &goal, const double &yaw)
{
    double goal_head;
    tf2::Quaternion q(goal->target_pose.pose.orientation.x, goal->target_pose.pose.orientation.y, goal->target_pose.pose.orientation.z, goal->target_pose.pose.orientation.w);
    double temp1, temp2, goal_yaw;
    tf2::Matrix3x3(q).getRPY(temp1, temp2, goal_yaw);
    goal_head = goal_yaw - yaw;

    return goal_head;
}