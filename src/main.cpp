#include <rubik_cube_solve/RubikCubeSolve.h>
#include <Eigen/Eigen>
#include <Eigen/Geometry>
#include <Eigen/Core>
#include <math.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
using namespace Eigen;

// #include <rubik_cube_solve/object_common.h
double angle2rad(double angle)
 {
     return (angle/180)*3.1415;
 }

void QtoE(double q1, double q2, double q3, double q0, double& A, double& B, double&C){
    A = atan2(2*(q0*q1+q2*q3), 1-2*(q1*q1+q2*q2));
    B = asin(2*(q0*q2-q1*q3));
    C = atan2(2*(q0*q3+q1*q2), 1-2*(q2*q2+q3*q3));
    // Eigen::Quaterniond q;
    // q.x()= q1;
    // q.y()= q2;
    // q.z()= q3;
    // q.w()= q0;
    // Eigen::Vector3d eular = q.toRotationMatrix().eulerAngles(2,1,0);
    // A = eular[0];
    // B = eular[1];
    // C = eular[2];
}

Eigen::Quaterniond euler2Quaternion(const double roll, const double pitch, const double yaw)
{
    Eigen::AngleAxisd rollAngle(roll, Eigen::Vector3d::UnitZ());
    Eigen::AngleAxisd pitchAngle(pitch, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd yawAngle(yaw, Eigen::Vector3d::UnitX());

    Eigen::Quaterniond q = rollAngle*pitchAngle*yawAngle;
    return q;
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


    // std::vector<geometry_msgs::Pose> waypoints;
    // geometry_msgs::PoseStamped target_pose3 = move_group1.getCurrentPose(move_group1.getEndEffectorLink());
    // std::vector<double> data  = move_group1.getCurrentRPY(move_group1.getEndEffectorLink());
    // std::cout << "x: "<<target_pose3.pose.orientation.x << " "<< "y: "<<target_pose3.pose.orientation.y << " "<< "z: "<<target_pose3.pose.orientation.z << " " \
    //     << "w: "<<target_pose3.pose.orientation.w << " "<<std::endl;

    // for(auto it :data)
    //     std::cout <<it<< " ";
    // std::cout<<std::endl;
    // double r, p,y ;
    // QtoE(target_pose3.pose.orientation.x, target_pose3.pose.orientation.y,target_pose3.pose.orientation.z,target_pose3.pose.orientation.w,r,p,y);
    // std::cout << "r "<<r<<" p "<<p<<" y "<<y<<std::endl;

    // tf::Quaternion q = tf::createQuaternionFromRPY(r,p,y +10);
    // target_pose3.pose.orientation.w = q.w();
    // target_pose3.pose.orientation.x = q.x();
    // target_pose3.pose.orientation.y = q.y();
    // target_pose3.pose.orientation.z = q.z();

    // geometry_msgs::Pose p1 = target_pose3.pose;
    // waypoints.push_back(p1);
    // moveit_msgs::RobotTrajectory trajectory;
    // const double jump_threshold = 0.0;
    // const double eef_step = 0.01;
    // double fraction = move_group1.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
    // moveit::planning_interface::MoveGroupInterface::Plan plan;
	// plan.trajectory_ = trajectory;
    // std::cout <<"fraction: "<<fraction<<std::endl;
    // if(fraction > 0.5)
    //     move_group1.execute(plan);


    // move_group1.setStartStateToCurrentState();
    // std::vector<double> joint = move_group1.getCurrentJointValues(); 
    // ROS_INFO_STREAM("joint_6 current:" << joint[5]);
    // double a = angle2rad(10.0);
    // joint[5] += a;
    // ROS_INFO_STREAM("joint_6 reference:" << joint[5]);
    // moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    // std::vector<std::string> data = move_group1.getJointNames();
    // for(auto it : data)
    // {
    //     std::cout << it <<" "<<std::endl;
    // }
    // move_group1.setJointValueTarget("R_joint_1",joint[0]); 
    // move_group1.setJointValueTarget("R_joint_2",joint[1]); 
    // move_group1.setJointValueTarget("R_joint_3",joint[2]); 
    // move_group1.setJointValueTarget("R_joint_4",joint[3]); 
    // move_group1.setJointValueTarget("R_joint_5",joint[4]); 
    // move_group1.setJointValueTarget("R_joint_6",joint[5]); 
    // if(move_group1.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS)
    // {
    //     move_group1.execute(my_plan);
    // }
    return 0;
}