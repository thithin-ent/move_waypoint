#include <obstacle_check/obstacle_check.h>

int main (int argc, char *argv[])
{   
    ros::init(argc, argv, "obstacle_check");
    Obstacle_check obstacle_check;
    ros::spin();    
    
    return 0;
}

