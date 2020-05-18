#include <rubik_cube_solve/RubikCubeSolve.h>
#include <Eigen/Eigen>
#include <Eigen/Geometry>
#include <Eigen/Core>
#include <math.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
using namespace Eigen;

void robotMoveCartesianUnit(moveit::planning_interface::MoveGroupInterface& group, double x, double y, double z)
{
    geometry_msgs::PoseStamped temp_pose = group.getCurrentPose();
    temp_pose.pose.position.x +=x;
    temp_pose.pose.position.y +=y;
    temp_pose.pose.position.z +=z;
    group.setPoseTarget(temp_pose);
    moveit_msgs::OrientationConstraint ocm;
    ocm.link_name = group.getEndEffectorLink();
    ocm.header.frame_id = "world";
    ocm.orientation.w = 1.0;
    ocm.absolute_x_axis_tolerance = 0.01;
    ocm.absolute_y_axis_tolerance = 0.01;
    ocm.absolute_z_axis_tolerance = 0.01;
    ocm.weight = 1.0;

    moveit_msgs::Constraints test_constraints;
    test_constraints.orientation_constraints.push_back(ocm);
    group.setPathConstraints(test_constraints);
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    while( group.plan(plan) != moveit::planning_interface::MoveItErrorCode::SUCCESS);
    group.move();
    group.clearPathConstraints();
}

void robotMoveCartesianUnit2(moveit::planning_interface::MoveGroupInterface& group, double x, double y, double z)
{
    geometry_msgs::PoseStamped temp_pose = group.getCurrentPose();
    const double jump_threshold = 0.0;
    const double eef_step = 0.01;

    temp_pose.pose.position.x +=x;
    temp_pose.pose.position.y +=y;
    temp_pose.pose.position.z +=z;

    geometry_msgs::Pose target_pose = temp_pose.pose;
    std::vector<geometry_msgs::Pose> waypoints;
    waypoints.push_back(target_pose);
    moveit_msgs::RobotTrajectory trajectory;
    while( group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory) < 0.8);
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    plan.trajectory_ = trajectory;
    group.execute(plan);
}



int main(int argc, char *argv[])
{
    ros::init(argc, argv, "rubik_cube_solve");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(4);
    spinner.start();
    moveit::planning_interface::MoveGroupInterface move_group0("arm0");
    moveit::planning_interface::MoveGroupInterface move_group1("arm1");
    RubikCubeSolve r(nh, move_group0, move_group1);
    r.spin();
    ros::waitForShutdown();

//    robotMoveCartesianUnit(move_group1, 0, 0, 0.02);
//    robotMoveCartesianUnit2(move_group1, 0.02, -0.02, 0.02);

    return 0;
}
