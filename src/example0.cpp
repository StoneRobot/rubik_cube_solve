#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/PoseStamped.h>

#include <vector>
#include <iostream>
#include <fstream>

bool TransformToWorldFrame(geometry_msgs::PoseStamped& poseStamped)
{
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    geometry_msgs::PoseStamped* worldFramePose = new geometry_msgs::PoseStamped[1];
    geometry_msgs::PoseStamped* otherFramePose = new geometry_msgs::PoseStamped[1];
    tf::TransformListener listener;

    otherFramePose[0] = poseStamped;
    for(int i=0; i < 5; ++i)
    {
        try
        {
            listener.transformPose("world", otherFramePose[0], worldFramePose[0]);
            break;
        }
        catch(tf::TransformException& ex)
        {
            ROS_INFO_STREAM(ex.what());
            ros::WallDuration(1).sleep();
            continue;
        }
    }
    poseStamped = worldFramePose[0];
    delete[] worldFramePose;
    delete[] otherFramePose;
    if(poseStamped.header.frame_id == "world")
    {
        return true;
    }
    else
    {
        return false;
    }
}

moveit::planning_interface::MoveItErrorCode moveGroupPlanAndMove(moveit::planning_interface::MoveGroupInterface& move_group, \
                                                                moveit::planning_interface::MoveGroupInterface::Plan my_plan)
{
    for(int i=0; i<5; i++)
    {
        if(move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS)
        {
            break;
        }
    }
    return move_group.move();
}

geometry_msgs::PoseStamped setEulerAngle(moveit::planning_interface::MoveGroupInterface& move_group, float x, float y, float z)
 {
    geometry_msgs::PoseStamped poseStamped;
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    // 设置姿态, 位置为0
    poseStamped.header.frame_id = move_group.getEndEffectorLink();
    tf2::Quaternion orientation;
    orientation.setEuler(y, x, z);
    poseStamped.pose.orientation = tf2::toMsg(orientation);
    TransformToWorldFrame(poseStamped);
    move_group.setPoseTarget(poseStamped);
    // 规划动作
    moveGroupPlanAndMove(move_group, my_plan);
    return poseStamped;
 }

geometry_msgs::PoseStamped setEndEffectorPositionTarget(moveit::planning_interface::MoveGroupInterface& move_group, float x, float y, float z)
{
    geometry_msgs::PoseStamped poseStamped;
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    // 设置位置
    poseStamped.header.frame_id = move_group.getEndEffectorLink();
    poseStamped.pose.position.x = x;
    poseStamped.pose.position.y = y;
    poseStamped.pose.position.z = z;
    poseStamped.pose.orientation.w = 1;
    TransformToWorldFrame(poseStamped);
    move_group.setPoseTarget(poseStamped);
    // 规划动作
    moveGroupPlanAndMove(move_group, my_plan);
    return poseStamped;
}

geometry_msgs::PoseStamped setEndEffectorPosoTarget(moveit::planning_interface::MoveGroupInterface& move_group, \
                                                                    float x, float y, float z,
                                                                    float X, float Y, float Z)
{
    geometry_msgs::PoseStamped poseStamped;
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    // 设置位置
    poseStamped.header.frame_id = move_group.getEndEffectorLink();
    poseStamped.pose.position.x = x;
    poseStamped.pose.position.y = y;
    poseStamped.pose.position.z = z;
    tf2::Quaternion orientation;
    orientation.setEuler(Y, X, Z);
    poseStamped.pose.orientation = tf2::toMsg(orientation);
    TransformToWorldFrame(poseStamped);
    move_group.setPoseTarget(poseStamped);
    moveGroupPlanAndMove(move_group, my_plan);
    return poseStamped;
}

 float angle2rad(float& angle)
 {
     return (angle/180)*3.1415;
 }

void setJointValue(moveit::planning_interface::MoveGroupInterface& move_group, float angle)
{
    std::vector<double> joint;
    joint = move_group.getCurrentJointValues();
    joint[5] += angle2rad(angle);
    move_group.setJointValueTarget(joint);
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    move_group.plan(my_plan);
    move_group.move();
}

std::vector<geometry_msgs::PoseStamped> poseStmpedS;
void getPoseStamped(moveit::planning_interface::MoveGroupInterface& move_group0, moveit::planning_interface::MoveGroupInterface& move_group1)
{
    move_group0.setNamedTarget("home0");
    move_group1.setNamedTarget("home1");
    move_group0.move();
    move_group1.move();   
    setEulerAngle(move_group0, -1.57, 0, 0);
    setEulerAngle(move_group1, 1.57, 0, 0);
    setEulerAngle(move_group0, 0, -1.57, 0);
    setEulerAngle(move_group1, 0, -1.57, 0);
    poseStmpedS.resize(8);
    // 
    poseStmpedS[0] = setEndEffectorPositionTarget(move_group0, 0.3, 0, 0.2);
    poseStmpedS[1] = setEndEffectorPositionTarget(move_group1, 0.3, 0, 0.2);
    //
    poseStmpedS[2] = setEndEffectorPositionTarget(move_group0, 0.17, 0, 0);
    poseStmpedS[3] = setEndEffectorPositionTarget(move_group1, 0.17, 0, 0);
    //
    poseStmpedS[4] = setEndEffectorPosoTarget(move_group0, 0, 0, 0.2, 0, -0.785, 0);
    poseStmpedS[5] = setEndEffectorPosoTarget(move_group1, 0, 0, 0.2, 0, -0.785, 0);

    moveit::planning_interface::MoveGroupInterface::Plan my_plan0;
    moveit::planning_interface::MoveGroupInterface::Plan my_plan1;
    move_group0.setPoseTarget(poseStmpedS[2]);
    moveGroupPlanAndMove(move_group0, my_plan0);
    move_group1.setPoseTarget(poseStmpedS[3]);
    moveGroupPlanAndMove(move_group1, my_plan1);


    // setEndEffectorPosoTarget(move_group0, 0, 0, 0, 0, 0.785, 0);
    // setEndEffectorPosoTarget(move_group1, 0, 0, 0, 0, 0.785, 0);
    // setEndEffectorPosoTarget(move_group0, 0, 0, -0.2, 0, 0, 0);
    // setEndEffectorPosoTarget(move_group1, 0, 0, -0.2, 0, 0, 0);
}

void setRecordPose(moveit::planning_interface::MoveGroupInterface& move_group, const geometry_msgs::PoseStamped& poseStamped)
{
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    move_group.setPoseTarget(poseStamped);
    move_group.plan(my_plan);
    move_group.move();
}

void example(moveit::planning_interface::MoveGroupInterface& move_group0, moveit::planning_interface::MoveGroupInterface& move_group1)
{
    setRecordPose(move_group0, poseStmpedS[0]);
    setRecordPose(move_group1, poseStmpedS[1]);
    setJointValue(move_group0, 90);
    setJointValue(move_group1, 90);
    setRecordPose(move_group0, poseStmpedS[2]);
    setRecordPose(move_group1, poseStmpedS[3]);
    setJointValue(move_group0, -90);
    setJointValue(move_group1, -90);
    setRecordPose(move_group0, poseStmpedS[4]);
    setRecordPose(move_group1, poseStmpedS[5]);
    setJointValue(move_group0, 180);
    setJointValue(move_group1, 180);
    setJointValue(move_group0, -180);
    setJointValue(move_group1, -180);
}

int  writePoseFile(std::string path)
{
    std::string filePath;
    std::ofstream fout(path, std::ios::out | std::ios::binary);
    if(!fout)
    {
        ROS_INFO_STREAM("save pose filed");
        return -1;
    }
    for(int i=0; i < poseStmpedS.size(); ++i)
    {
        fout.write((const char*)&poseStmpedS[i], sizeof(poseStmpedS[i]));
    }
    fout.close();
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "only_joint_action");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(2);
    spinner.start();
    moveit::planning_interface::MoveGroupInterface move_group("arm");
    geometry_msgs::PoseStamped pose = move_group.getCurrentPose();
    std::vector<double> joint = move_group.getCurrentJointValues();
    joint[0] += 0.1;
    // move_group.setJointValueTarget(joint);
    ROS_INFO_STREAM(pose);
    pose.pose.position.x += 0.03;
    move_group.setPoseTarget(pose);
    ROS_INFO_STREAM(pose);
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    move_group.setPlanningTime(10);
    bool flag;
    do
    {
        flag = (move_group.plan(my_plan)  == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        /* code */
    } while (!flag && ros::ok());
    move_group.move();
    ros::waitForShutdown();

    // moveit::planning_interface::MoveGroupInterface move_group1("arm1");
    // std::string posesFilePath;
    // nh.getParam("/poses_file_path", posesFilePath);
    // getPoseStamped(move_group0, move_group1);
    // long cnt = 0;
    // std::cin.ignore();
    // writePoseFile(posesFilePath);
    // while (ros::ok())
    // {
    //     example(move_group0, move_group1);
    // }
    return 0;
}