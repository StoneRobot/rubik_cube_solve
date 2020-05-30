#include <ros/ros.h>
#include <string>
#include "rubik_cube_solve/object_common.h"


int main(int argc, char *argv[])
{
    ros::init(argc, argv, "add");
    ros::NodeHandle nh;
    // moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    ros::Publisher planning_scene_diff_publisher = nh.advertise<moveit_msgs::PlanningScene>("planning_scene", 1);
    while (planning_scene_diff_publisher.getNumSubscribers() < 1)
     {
        ros::Duration(0.5).sleep();
    }
    moveit_msgs::PlanningScene p;
    bool isInvisible;
    std::istringstream(argv[12]) >> std::boolalpha >> isInvisible;
    if(argc == 13)
        addCollisionObjects(planning_scene_diff_publisher, p,
        atof(argv[1]), atof(argv[2]), atof(argv[3]), 
        atof(argv[4]), atof(argv[5]), atof(argv[6]), 
        atof(argv[7]), atof(argv[8]), atof(argv[9]), 
        argv[10], argv[11], isInvisible);
    else if(argc == 12)
        addCollisionObjects(planning_scene_diff_publisher, p,
        atof(argv[1]), atof(argv[2]), atof(argv[3]), 
        atof(argv[4]), atof(argv[5]), atof(argv[6]), 
        atof(argv[7]), atof(argv[8]), atof(argv[9]), 
        argv[10], argv[11]);
    // ros::waitForShutdown();
    return 0;
}
