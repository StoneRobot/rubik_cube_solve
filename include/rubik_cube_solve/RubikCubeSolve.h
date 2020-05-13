#pragma once
#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_srvs/Empty.h>
#include <std_msgs/Int8MultiArray.h>

#include "rubik_cube_solve/rubik_cube_solve_cmd.h"
#include "rubik_cube_solve/end_effector_motion.h"
#include "rubik_cube_solve/recordPoseStamped.h"
#include "hirop_msgs/openGripper.h"
#include "hirop_msgs/closeGripper.h"
#include "cubeParse/TakePhoto.h"
#include "cubeParse/SolveCube.h"
#include "std_msgs/Bool.h"

#include <vector>
#include <iostream>
#include <fstream>
#include <stdlib.h>
#include "yaml-cpp/yaml.h"

// 記錄三個空間點的以末端坐標系的数据变化
struct RecordPoseData
{
    double pose0[6];
    double pose1[6];
    double pose2[6];
};

struct CubeState
{
    // 0号机或1号机
    int captureRobot;
    // 1,2,3,4个点
    int capturePoint;
    // 在一号空间能拧动的位置
    int canRotateFace1;
    // 在二号空间能拧动的位置
    int canRotateFace2;
    bool isFinish = false;
};

struct ActionData
{
    int captureRobot;
    int otherRobot;
    int capturePoint;
    int space;
    double angle;
    bool isSwop;
};

class RubikCubeSolve
{
public:
    RubikCubeSolve(ros::NodeHandle nh, moveit::planning_interface::MoveGroupInterface& group0, moveit::planning_interface::MoveGroupInterface& group1);
    // 將坐標系轉換到世界坐標系
    bool transformFrame(geometry_msgs::PoseStamped& poseStamped, std::string frame_id);
    // 多次規劃和move, 減少失敗的概率
    moveit::planning_interface::MoveItErrorCode moveGroupPlanAndMove(moveit::planning_interface::MoveGroupInterface& move_group, \
                                                                moveit::planning_interface::MoveGroupInterface::Plan& my_plan);
    // 設置歐拉角
    geometry_msgs::PoseStamped setEulerAngle(moveit::planning_interface::MoveGroupInterface& move_group, double x, double y, double z, bool radian);
    // 以末端爲參考系,進行移動
    geometry_msgs::PoseStamped setEndEffectorPositionTarget(moveit::planning_interface::MoveGroupInterface& move_group, double x, double y, double z);
    // 先移動,後旋轉
    geometry_msgs::PoseStamped setEndEffectorPosoTarget(moveit::planning_interface::MoveGroupInterface& move_group, \
                                                        double x, double y, double z,\
                                                        double X, double Y, double Z, bool radian, bool only_get_pose);
    // 角度轉爲弧度
    double angle2rad(double& angle);
    // 設置joint_6的角度
    void setJoint6Value(moveit::planning_interface::MoveGroupInterface& move_group, int angle);
    // 循環執行,防止控制器不響應
    moveit::planning_interface::MoveItErrorCode loop_move(moveit::planning_interface::MoveGroupInterface& move_group);
    // 一步
    bool step(moveit::planning_interface::MoveGroupInterface& capture_move_group,\
                moveit::planning_interface::MoveGroupInterface& rotate_move_group,\
                geometry_msgs::PoseStamped pose, int space_point, int angle);
    bool swop(moveit::planning_interface::MoveGroupInterface& capture_move_group,\
                        moveit::planning_interface::MoveGroupInterface& rotate_move_group,\
                        geometry_msgs::PoseStamped pose);
    bool rotateCube(moveit::planning_interface::MoveGroupInterface& rotate_move_group, 
                    geometry_msgs::PoseStamped pose,
                    int angle);
    moveit::planning_interface::MoveItErrorCode setAndMove(moveit::planning_interface::MoveGroupInterface& move_group, \
                                                                        geometry_msgs::PoseStamped& );
    // 将面和角度映射到几号机器人抓住什么点位,并拧转多少度
    void analyseData(int face, int angle);

    // ~RubikCubeSolve();

    // 記錄從 getPoseStamped() 獲取的點
    int  writePoseFile();
    bool writePoseOnceFile(const std::string& name, const geometry_msgs::PoseStamped& pose);
    // 現場獲取N個坐標
    void getPoseStamped();
    // 從文件讀取N個坐標
    inline void loadRobotPoses();
    void loadRobotPose(std::string path, int row, int column);
    void loadPickPose();
    
    void action();

    void loadPoseData();

    void initPose();
    void addData(geometry_msgs::PoseStamped& pose, YAML::Node doc);
    void addData(double pose[], const std::string& num_of_pose, YAML::Node doc);

    void example();
    void loop();


    bool openGripper(moveit::planning_interface::MoveGroupInterface& move_group);
    bool closeGripper(moveit::planning_interface::MoveGroupInterface& move_group);

    void setOrientationConstraints(moveit::planning_interface::MoveGroupInterface& move_group, \
                                                    double x_axis_tolerance=6.3, double y_axis_tolerance=6.3,double z_axis_tolerance=0.8);
    void setPositionConstraints(moveit::planning_interface::MoveGroupInterface& group);

    void setJointConstraints(moveit::planning_interface::MoveGroupInterface& group);
    void setConstraints(moveit::planning_interface::MoveGroupInterface& move_group, \
                                                    double x_axis_tolerance=6.3, double y_axis_tolerance=6.3,double z_axis_tolerance=0.8);

    void clearConstraints(moveit::planning_interface::MoveGroupInterface& group);

    void goPreparePose();

    void photograph();
    void photographPickPlace(geometry_msgs::PoseStamped& pose, bool isPick);
    void shoot(int num);
    void stopMove();
    void spin();
private:
    void placeCube();
    
    void getPrepareSomeDistanceRobotPose();

    inline void getPrepareSomeDistance(std::vector<std::vector<geometry_msgs::PoseStamped> >& pose, int row, int column);
    void fakeInitializationState();
    moveit::planning_interface::MoveGroupInterface& getMoveGroup(int num);

    std::vector<std::vector<geometry_msgs::PoseStamped> > robotPose;
    const int ROWS = 2;
    const int COLUMNS = 8;

    std::vector<geometry_msgs::PoseStamped> photographPose;
    moveit::planning_interface::MoveGroupInterface& move_group0;
    moveit::planning_interface::MoveGroupInterface& move_group1;

    ros::NodeHandle nh;
    ros::ServiceServer analyseCmd;
    bool analyseCallBack(rubik_cube_solve::rubik_cube_solve_cmd::Request& req, rubik_cube_solve::rubik_cube_solve_cmd::Response& rep);
    ros::ServiceServer end_effector;
    bool endEffectorPoseCallBack(rubik_cube_solve::end_effector_motion::Request& req, rubik_cube_solve::end_effector_motion::Response& rep);
    ros::ServiceServer record_pose;
    bool recordPoseCallBack(rubik_cube_solve::recordPoseStamped::Request& req, rubik_cube_solve::recordPoseStamped::Response& rep);
    ros::ServiceServer stop_move;
    bool sotpMoveCallBack(std_srvs::Empty::Request& req, std_srvs::Empty::Response& rep);


    ros::ServiceClient receiveSolve;
    ros::ServiceClient shootClient;

    ros::Subscriber beginSolve;
    void beginSolve_sub(const std_msgs::BoolConstPtr& msg);
    bool isBegingSolve = false;

    ros::Subscriber rubikCubeSolveData_sub;
    void rubikCubeSolveDataCallBack(const std_msgs::Int8MultiArrayConstPtr& msg);
    void transformData();
    std::vector<int> rubikCubeSolveData;
    std::vector<std::vector<int> > rubikCubeSolvetransformData;

    
    ros::ServiceClient openGripper_client0;
    ros::ServiceClient closeGripper_client0;
    ros::ServiceClient openGripper_client1;
    ros::ServiceClient closeGripper_client1;

    RecordPoseData data0;
    RecordPoseData data1;

    CubeState Cstate;
    ActionData Adata;
    double prepare_some_distance;
    const double rubikCubeAdd = 0.0095;
    const int UP = 1;
    const int down = 2;
};