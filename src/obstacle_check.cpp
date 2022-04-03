#include <obstacle_check/obstacle_check.h>

Obstacle_check::Obstacle_check()
{
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);
    geometry_msgs::TransformStamped transformStamped;
    try
    {
        while (!tfBuffer.canTransform("base_link", "laser", ros::Time(0), ros::Duration(0.1)))
        {
            ROS_INFO("%s", "ready to listen base2laser ...");
        }
        transformStamped = tfBuffer.lookupTransform("base_link", "laser", ros::Time(0));
    }
    catch (tf2::TransformException ex)
    {
        // ROS_ERROR("%s", ex.what());
    }
    tf2::Quaternion q(transformStamped.transform.rotation.x, transformStamped.transform.rotation.y, transformStamped.transform.rotation.z, transformStamped.transform.rotation.w);
    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
    transform_ << cos(yaw), -sin(yaw), transformStamped.transform.translation.x, sin(yaw), cos(yaw), transformStamped.transform.translation.y, 0, 0, 1;

    nh_.param<double>("/obstacle_radius", radius_param_,0.5);
    nh_.param<double>("/obstacle_radian", radian_param_,0.3);

    scan_sub_ = nh_.subscribe("scan", 1, &Obstacle_check::scan_callback, this);
    basescan_pub_ = nh_.advertise<sensor_msgs::PointCloud>("basepointcloud", 1);
    obCheck_pub_ = nh_.advertise<std_msgs::Bool>("obstacle_check", 1);
}

Obstacle_check::~Obstacle_check()
{
}

void Obstacle_check::scan_callback(const sensor_msgs::LaserScanConstPtr &data)
{
    int index_max = (data->angle_max - data->angle_min) / data->angle_increment;

    std::vector<Vector3f> scan_poses;

    for (int i = 0; i < index_max; i++)
    {
        float range_temp = data->ranges[i];
        if (range_temp < 0.1)
        {
            continue;
        }

        float angle_temp = data->angle_min + i * data->angle_increment;
        Vector3f scan_pose, temp;
        scan_pose << range_temp * cos(angle_temp), range_temp * sin(angle_temp), 1.0;
        scan_pose = transform_ * scan_pose;
        scan_poses.push_back(scan_pose);
        //cout << "scan_poses: " << scan_pose(0) << "  " << scan_pose(1) << endl;
    }
    basescan_make(scan_poses);
    std_msgs::Bool checkobstacle;
    checkobstacle.data = obstacle_check(scan_poses);
    obCheck_pub_.publish(checkobstacle);
}

void Obstacle_check::basescan_make(const vector<Vector3f> &scan_poses)
{
    sensor_msgs::PointCloud basepoint;
    int index = scan_poses.size();
    vector<geometry_msgs::Point32> points;
    vector<sensor_msgs::ChannelFloat32> Channels;
    basepoint.header.stamp = ros::Time::now();
    basepoint.header.frame_id = "base_link";
    for (int i = 0; i < index; i++)
    {
        geometry_msgs::Point32 point;
        sensor_msgs::ChannelFloat32 Channel;
        point.x = scan_poses[i](0);
        point.y = scan_poses[i](1);
        point.z = 0.2;
        points.push_back(point);
        Channels.push_back(Channel);
    }
    basepoint.points = points;
    basepoint.channels = Channels;
    basescan_pub_.publish(basepoint);
}

bool Obstacle_check::obstacle_check(const vector<Vector3f> &scan_poses)
{
    int index = scan_poses.size();

    for (int i = 0; i < index; i++)
    {
        double range = sqrt(pow(scan_poses[i](0),2) + pow(scan_poses[i](1),2));
        double angle = atan2(scan_poses[i](1), scan_poses[i](0));
        //cout << "rnage: " << range << "  " << angle << endl;
        if (range < radius_param_ && abs(angle) < radian_param_){
            return true;
            //cout << "scan_poses: " << scan_poses[i](0) << "  " << scan_poses[i](1) << endl;
        }
    }
    return false;
}