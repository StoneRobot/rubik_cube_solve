#include <rubik_cube_solve/RubikCubeSolve.h>

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "rubik_cube_solve");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(4);
    spinner.start();
    moveit::planning_interface::MoveGroupInterface move_group0("arm0");
    moveit::planning_interface::MoveGroupInterface move_group1("arm1");
    RubikCubeSolve r(nh, move_group0, move_group1);
    ros::waitForShutdown();
    return 0;
}