#include <rubik_cube_solve/RubikCubeSolve.h>

RubikCubeSolve::RubikCubeSolve(ros::NodeHandle nodehandle, \
                                moveit::planning_interface::MoveGroupInterface& group0, \
                                moveit::planning_interface::MoveGroupInterface& group1)
:move_group0{group0},
move_group1{group1}
{
    nh = nodehandle;
    nh.getParam("/rubik_cube_solve/prepare_some_distance", prepare_some_distance);
    analyseCmd = nh.advertiseService("analyse_rubik_cube_cmd", &RubikCubeSolve::analyseCallBack, this);
    end_effector = nh.advertiseService("end_effector_pose", &RubikCubeSolve::endEffectorPoseCallBack, this);
    record_pose = nh.advertiseService("record_pose", &RubikCubeSolve::recordPoseCallBack, this);
    stop_move = nh.advertiseService("stop_move", &RubikCubeSolve::sotpMoveCallBack, this);
    openGripper_client0 = nh.serviceClient<hirop_msgs::openGripper>("openGripper0");
    closeGripper_client0 = nh.serviceClient<hirop_msgs::closeGripper>("closeGripper0");
    openGripper_client1 = nh.serviceClient<hirop_msgs::openGripper>("openGripper1");
    closeGripper_client1 = nh.serviceClient<hirop_msgs::closeGripper>("closeGripper1");
    robotPose.resize(2);
    robotPose[0].resize(8);
    robotPose[1].resize(8);
    photographPose.resize(3);

    loadPickPose();
    photograph();

    loadPoseData();
    initPose();

    goPreparePose();
}

void RubikCubeSolve::stopMove()
{
    move_group0.stop();
    move_group1.stop();
}

void RubikCubeSolve::goPreparePose()
{
    setAndMove(move_group0, robotPose[0][0]);
    setAndMove(move_group1, robotPose[1][0]);

    bool fake;
    nh.getParam("/rubik_cube_solve/fakeInitializationState",fake);
    if(fake)
        fakeInitializationState();

}

void RubikCubeSolve::photographPickPlace(geometry_msgs::PoseStamped& pose, bool isPick)
{
    setAndMove(move_group1, pose);
    if(isPick)
    {
        openGripper(move_group1);
        setEndEffectorPositionTarget(move_group1, prepare_some_distance, 0, 0);
        closeGripper(move_group1);
        setEndEffectorPositionTarget(move_group1, -prepare_some_distance, 0, 0);
        move_group1.setNamedTarget("home1");
        loop_move(move_group1);
        setEulerAngle(move_group1, -90, 0, 0, false);
    }
    else
    {
        setEndEffectorPositionTarget(move_group1, prepare_some_distance, 0, 0);
        openGripper(move_group1);
        setEndEffectorPositionTarget(move_group1, -prepare_some_distance, 0, 0);
    }
}

void RubikCubeSolve::photograph()
{
    move_group0.setNamedTarget("home0");
    move_group1.setNamedTarget("home1");
    loop_move(move_group0);
    loop_move(move_group1);

    photographPickPlace(photographPose[0], true);
    // 拍摄者到达拍摄位置
    setEulerAngle(move_group0, -90, 0, 0, false);
    setEulerAngle(move_group0, 0, -90, 0, false);
    //  第一张
    shoot(0);

    // 第二张
    setEulerAngle(move_group1, 180, 0, 0, false);
    shoot(0);
    
    // 放置换面
    photographPickPlace(photographPose[0], false);
    geometry_msgs::PoseStamped pose;
    pose = setEndEffectorPosoTarget(move_group1, 0, rubikCubeAdd, -rubikCubeAdd, -90, 0, 0, false);
    photographPickPlace(pose, true);
    // 第三张
    shoot(0);
    // 第四张
    setEulerAngle(move_group1, 180, 0, 0, false);
    shoot(0);
    // 更换抓取的机器人
    photographPickPlace(photographPose[0], false);
    move_group1.setNamedTarget("home1");
    loop_move(move_group1);
    // 到达拍照位置
    setEulerAngle(move_group1, 90, 0, 0, false);
    setEulerAngle(move_group1, 0, -90, 0, false);
    // 0号机去抓取
    setAndMove(move_group0, photographPose[1]);
    openGripper(move_group0);
    setEndEffectorPositionTarget(move_group0, prepare_some_distance, 0, 0);
    closeGripper(move_group0);
    setEndEffectorPositionTarget(move_group0, -prepare_some_distance, 0, 0);
    move_group0.setNamedTarget("home0");
    loop_move(move_group0);
    setEulerAngle(move_group0, -90, 0, 0, false);
    // 第五张
    shoot(1);
    setEulerAngle(move_group0, 180, 0, 0, false);
    // 第六张
    shoot(1);
}

void RubikCubeSolve::shoot(int num)
{
    ROS_INFO_STREAM("shoot");
    ros::WallDuration(2.0).sleep();
}

void RubikCubeSolve::fakeInitializationState()
{
    Cstate.captureRobot = 0;
    Cstate.capturePoint = 3;
    Cstate.canRotateFace1 = 5;
    Cstate.canRotateFace2 = 1;
    Adata.captureRobot = Cstate.captureRobot;
    Adata.capturePoint = Cstate.capturePoint;
    Adata.otherRobot = (Cstate.captureRobot + 1)%2;
    geometry_msgs::PoseStamped pose = robotPose[0][Cstate.capturePoint];
    pose.pose.position.y += prepare_some_distance;
    setAndMove(move_group0, pose);
    setAndMove(move_group1, robotPose[1][0]);
}

void RubikCubeSolve::setOrientationConstraints(moveit::planning_interface::MoveGroupInterface& move_group)
{
    moveit_msgs::OrientationConstraint ocm;
    ocm.link_name = move_group.getEndEffectorLink();
    ocm.header.frame_id = "world";
    ocm.orientation = move_group.getCurrentPose().pose.orientation;
    ROS_INFO_STREAM(move_group.getCurrentPose());
    ocm.absolute_x_axis_tolerance = 6.3;
    ocm.absolute_y_axis_tolerance = 6.3;
    ocm.absolute_z_axis_tolerance = 1.0;
    ocm.weight = 1.0;
    moveit_msgs::Constraints con;
    con.orientation_constraints.push_back(ocm);
    move_group.setPathConstraints(con);
    move_group.setPlanningTime(5.0);
}

void RubikCubeSolve::setPositionConstraints(moveit::planning_interface::MoveGroupInterface& move_group)
{
    moveit_msgs::PositionConstraint posi;
    posi.link_name = move_group.getEndEffectorLink();
    posi.header.frame_id = "world";
    posi.target_point_offset.x = 0.01;
    posi.target_point_offset.y = 0.01;
    posi.target_point_offset.z = 0.01;
    posi.constraint_region.primitives.resize(1);
    posi.constraint_region.primitives[0].type = posi.constraint_region.primitives[0].BOX;
    posi.constraint_region.primitives[0].dimensions.resize(3);
    posi.constraint_region.primitives[0].dimensions[0] = 0.20;
    posi.constraint_region.primitives[0].dimensions[1] = 0.45;
    posi.constraint_region.primitives[0].dimensions[2] = 0.6;
    posi.constraint_region.primitive_poses.resize(1);
    posi.constraint_region.primitive_poses[0].position = move_group.getCurrentPose().pose.position;
    posi.constraint_region.primitive_poses[0].orientation.w = 1;
    posi.weight = 1;
    moveit_msgs::Constraints con;
    con.position_constraints.push_back(posi);
    move_group.setPathConstraints(con);
    move_group.setPlanningTime(2.0);
}

void RubikCubeSolve::clearConstraints(moveit::planning_interface::MoveGroupInterface& move_group)
{
    move_group.clearPathConstraints();
}

moveit::planning_interface::MoveGroupInterface& RubikCubeSolve::getMoveGroup(int num)
{
    if(num == 0)
        return move_group0;
    else if (num == 1)
        return move_group1;
}

bool RubikCubeSolve::analyseCallBack(rubik_cube_solve::rubik_cube_solve_cmd::Request& req, rubik_cube_solve::rubik_cube_solve_cmd::Response& rep)
{
    if(req.face > 6 || req.face <= 0)
        Cstate.isFinish = true;
    else
    {
        analyseData(req.face, req.angle);
        action();
    }
    rep.isFinish = true;
    return true;
}

bool  RubikCubeSolve::endEffectorPoseCallBack(rubik_cube_solve::end_effector_motion::Request& req, rubik_cube_solve::end_effector_motion::Response& rep)
{
    geometry_msgs::PoseStamped pose;
    pose = setEndEffectorPosoTarget(getMoveGroup(req.group), req.x, req.y, req.z, req.Ex, req.Ey, req.Ez, false);
    if(pose.header.frame_id.empty())
    {
        rep.isFinish = false;
    }
    else
    {
        rep.isFinish = true;
    }
    return rep.isFinish;
}

bool RubikCubeSolve::recordPoseCallBack(rubik_cube_solve::recordPoseStamped::Request& req, rubik_cube_solve::recordPoseStamped::Response& rep)
{
    std::string path;
    nh.getParam("/rubik_cube_solve/record_pose_path", path);
    path += "/recordPose/" + req.PoseName;
    if(req.isJointSpace)
    {
        ROS_DEBUG("record joint space in development ...");
        rep.isFinish = false;
    }
    else
    {
        geometry_msgs::PoseStamped pose;
        pose = getMoveGroup(req.robot).getCurrentPose();
        writePoseOnceFile(path, pose);
        rep.isFinish = true;
    }
    return true;
}

bool RubikCubeSolve::sotpMoveCallBack(std_srvs::Empty::Request& req, std_srvs::Empty::Response& rep)
{
    stopMove();
    return true;
}

void RubikCubeSolve::analyseData(int face, int angle)
{
    
    if(face == Cstate.canRotateFace1)
    {
        Adata.space = 1;
        Adata.isSwop = false;
    }
    else if (face == Cstate.canRotateFace2)
    {
        Adata.space = 2;
        Adata.isSwop = false;
    }
    else
    {

        switch (face)
        {
            case 1:
                Adata.capturePoint = Cstate.capturePoint % 4 + 1;
                Adata.space = 2;
                Cstate.canRotateFace1 = Adata.capturePoint + 2;
                Cstate.canRotateFace2 = 1;
                break;
            case 2:
                Adata.capturePoint = Cstate.capturePoint % 4 + 1;;
                Adata.space = 2;
                Cstate.canRotateFace1 = Adata.capturePoint + 2;
                Cstate.canRotateFace2 = 2;
                break;
            case 3:
                Adata.capturePoint = 1;
                Adata.space = 1;
                Cstate.canRotateFace1 = 3;
                Cstate.canRotateFace2 = Cstate.canRotateFace2 % 2 + 1;
                break;
            case 4:
                Adata.capturePoint = 2;
                Adata.space = 1;
                Cstate.canRotateFace1 = 4;
                Cstate.canRotateFace2 = Cstate.canRotateFace2 % 2 + 1;
                break;
            case 5:
                Adata.capturePoint = 3;
                Adata.space = 1;
                Cstate.canRotateFace1 = 5;
                Cstate.canRotateFace2 = Cstate.canRotateFace2 % 2 + 1;
                break;
            case 6:
                Adata.capturePoint = 4;
                Adata.space = 1;
                Cstate.canRotateFace1 = 6;
                Cstate.canRotateFace2 = Cstate.canRotateFace2 % 2 + 1;
                break;
        }
        Adata.isSwop = true;
        Adata.captureRobot = (Cstate.captureRobot + 1)%2;
        Cstate.captureRobot = Adata.captureRobot;
        Cstate.capturePoint = Adata.capturePoint;
    }
    Adata.angle = angle;
    Adata.otherRobot = (Adata.captureRobot + 1)%2;
}

void RubikCubeSolve::action()
{
    if(Adata.isSwop)
    {
        swop(getMoveGroup(Adata.captureRobot), getMoveGroup(Adata.otherRobot), robotPose[Adata.captureRobot][Adata.capturePoint]);
    }
    step(getMoveGroup(Adata.captureRobot), getMoveGroup(Adata.otherRobot), robotPose[Adata.captureRobot][Adata.capturePoint], Adata.space, Adata.angle);
}

bool RubikCubeSolve::swop(moveit::planning_interface::MoveGroupInterface& capture_move_group,\
                        moveit::planning_interface::MoveGroupInterface& rotate_move_group,\
                        geometry_msgs::PoseStamped pose)
{
    openGripper(capture_move_group);
    setAndMove(capture_move_group, pose);
    // 到达
    setEndEffectorPositionTarget(capture_move_group, prepare_some_distance, 0, 0);
    closeGripper(capture_move_group);

    openGripper(rotate_move_group);
    setEndEffectorPositionTarget(rotate_move_group, -prepare_some_distance, 0, 0);
    setAndMove(rotate_move_group, robotPose[Adata.otherRobot][0]);
}

bool RubikCubeSolve::step(moveit::planning_interface::MoveGroupInterface& capture_move_group,\
                moveit::planning_interface::MoveGroupInterface& rotate_move_group,\
                geometry_msgs::PoseStamped pose, int space_point, int angle)
{
    if(space_point == 1)
    {
        setOrientationConstraints(capture_move_group);
        setOrientationConstraints(rotate_move_group);
        setAndMove(capture_move_group, robotPose[Adata.captureRobot][6]);

        clearConstraints(rotate_move_group);
        rotateCube(rotate_move_group, robotPose[Adata.otherRobot][7], angle);
    }
    else
    {
        rotateCube(rotate_move_group, robotPose[Adata.otherRobot][5], angle);
    }
    // 拧魔方的回原位
    setEndEffectorPositionTarget(rotate_move_group, -prepare_some_distance, 0, 0);
    setAndMove(rotate_move_group, robotPose[Adata.otherRobot][0]);
    // 抓住魔方的回原位
    pose.pose.position.y += pow(-1, Adata.captureRobot)*prepare_some_distance;
    setAndMove(capture_move_group, pose);
    clearConstraints(capture_move_group);
}

bool RubikCubeSolve::rotateCube(moveit::planning_interface::MoveGroupInterface& rotate_move_group, 
                                geometry_msgs::PoseStamped pose, int angle)
{
    openGripper(rotate_move_group);
    setAndMove(rotate_move_group, pose);
    setEndEffectorPositionTarget(rotate_move_group, prepare_some_distance, 0, 0);
    closeGripper(rotate_move_group);
    setJoint6Value(rotate_move_group, angle);
}

bool RubikCubeSolve::openGripper(moveit::planning_interface::MoveGroupInterface& move_group)
{
    bool flag;
    hirop_msgs::openGripper srv;
    if(move_group.getName() == "arm0")
        flag = openGripper_client0.call(srv);
    else
    {
        flag = openGripper_client1.call(srv);
    }
    return flag;
}
bool RubikCubeSolve::closeGripper(moveit::planning_interface::MoveGroupInterface& move_group)
{
    bool flag;
    hirop_msgs::closeGripper srv;
    if(move_group.getName() == "arm0")
        flag = closeGripper_client0.call(srv);
    else
    {
        flag = closeGripper_client1.call(srv);
    }
    return flag;
}

void RubikCubeSolve::initPose()
{
    bool is_read_pose;
    nh.getParam("/rubik_cube_solve/is_load_pose", is_read_pose);
    if(is_read_pose)
    {
        this->loadRobotPose();

    }
    else
    {
        this->getPoseStamped();
        writePoseFile();
    }
}

void RubikCubeSolve::loadRobotPose()
{
    std::string path;
    YAML::Node doc;

    nh.getParam("/pose03_path", path);
    doc = YAML::LoadFile(path);
    addData(robotPose[0][0], doc);

    nh.getParam("/pose021_path", path);
    doc = YAML::LoadFile(path);
    addData(robotPose[0][1], doc);

    nh.getParam("/pose022_path", path);
    doc = YAML::LoadFile(path);
    addData(robotPose[0][2], doc);

    nh.getParam("/pose023_path", path);
    doc = YAML::LoadFile(path);
    addData(robotPose[0][3], doc);

    nh.getParam("/pose024_path", path);
    doc = YAML::LoadFile(path);
    addData(robotPose[0][4], doc);

    nh.getParam("/pose025_path", path);
    doc = YAML::LoadFile(path);
    addData(robotPose[0][5], doc);

    nh.getParam("/pose011_path", path);
    doc = YAML::LoadFile(path);
    addData(robotPose[0][6], doc);

    nh.getParam("/pose012_path", path);
    doc = YAML::LoadFile(path);
    addData(robotPose[0][7], doc);
//////////////////////////////////////////////
    nh.getParam("/pose14_path", path);
    doc = YAML::LoadFile(path);
    addData(robotPose[1][0], doc);

    nh.getParam("/pose121_path", path);
    doc = YAML::LoadFile(path);
    addData(robotPose[1][1], doc);

    nh.getParam("/pose122_path", path);
    doc = YAML::LoadFile(path);
    addData(robotPose[1][2], doc);

    nh.getParam("/pose123_path", path);
    doc = YAML::LoadFile(path);
    addData(robotPose[1][3], doc);

    nh.getParam("/pose124_path", path);
    doc = YAML::LoadFile(path);
    addData(robotPose[1][4], doc);

    nh.getParam("/pose125_path", path);
    doc = YAML::LoadFile(path);
    addData(robotPose[1][5], doc);

    nh.getParam("/pose111_path", path);
    doc = YAML::LoadFile(path);
    addData(robotPose[1][6], doc);

    nh.getParam("/pose112_path", path);
    doc = YAML::LoadFile(path);
    addData(robotPose[1][7], doc);
}

void RubikCubeSolve::loadPickPose()
{
    std::string path;
    YAML::Node doc;

    nh.getParam("/pick0_path", path);
    doc = YAML::LoadFile(path);
    addData(photographPose[0], doc);
    photographPose[0].pose.position.z += prepare_some_distance;

    nh.getParam("/pick1_path", path);
    doc = YAML::LoadFile(path);
    addData(photographPose[1], doc);
    photographPose[1].pose.position.y -= prepare_some_distance;
}

void RubikCubeSolve::addData(geometry_msgs::PoseStamped& pose, YAML::Node node)
{
    pose.header.frame_id = node["header"]["frame_id"].as<std::string>();
    pose.pose.position.x = node["pose"]["position"]["x"].as<double>();
    pose.pose.position.y = node["pose"]["position"]["y"].as<double>();
    pose.pose.position.z = node["pose"]["position"]["z"].as<double>();
    pose.pose.orientation.x = node["pose"]["orientation"]["x"].as<double>();
    pose.pose.orientation.y = node["pose"]["orientation"]["y"].as<double>();
    pose.pose.orientation.z = node["pose"]["orientation"]["z"].as<double>();
    pose.pose.orientation.w = node["pose"]["orientation"]["w"].as<double>();
}

void RubikCubeSolve::addData(double pose[], const std::string& num_of_pose, YAML::Node doc)
{
    pose[0] = doc[num_of_pose]["x"].as<double>();
    pose[1] = doc[num_of_pose]["y"].as<double>();
    pose[2] = doc[num_of_pose]["z"].as<double>();
    pose[3] = doc[num_of_pose]["X"].as<double>();
    pose[4] = doc[num_of_pose]["Y"].as<double>();
    pose[5] = doc[num_of_pose]["Z"].as<double>();
}

void RubikCubeSolve::loadPoseData()
{
    std::string path;
    YAML::Node node;
    nh.getParam("/pose_data0_path", path);
    node = YAML::LoadFile(path);
    addData(data0.pose0, "pose0", node);
    addData(data0.pose1, "pose1", node);
    addData(data0.pose2, "pose2", node);

    nh.getParam("/pose_data1_path", path);
    node = YAML::LoadFile(path);
    addData(data1.pose0, "pose0", node);
    addData(data1.pose1, "pose1", node);
    addData(data1.pose2, "pose2", node);
}

bool RubikCubeSolve::TransformToWorldFrame(geometry_msgs::PoseStamped& poseStamped)
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

moveit::planning_interface::MoveItErrorCode RubikCubeSolve::setAndMove(moveit::planning_interface::MoveGroupInterface& move_group, \
                                                                        geometry_msgs::PoseStamped& poseStamped)
{
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    move_group.setPoseTarget(poseStamped);
    this->moveGroupPlanAndMove(move_group, my_plan);
}

moveit::planning_interface::MoveItErrorCode RubikCubeSolve::moveGroupPlanAndMove(moveit::planning_interface::MoveGroupInterface& move_group, \
                                                                moveit::planning_interface::MoveGroupInterface::Plan& my_plan)
{
    for(int i=0; i<5; i++)
    {
        if(move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS)
        {
            break;
        }
    }
    return loop_move(move_group);
}

geometry_msgs::PoseStamped RubikCubeSolve::setEulerAngle(moveit::planning_interface::MoveGroupInterface& move_group, double x, double y, double z, bool radian=true)
{
return setEndEffectorPosoTarget(move_group, 0, 0, 0, x, y, z, radian);
}

geometry_msgs::PoseStamped RubikCubeSolve::setEndEffectorPositionTarget(moveit::planning_interface::MoveGroupInterface& move_group, double x, double y, double z)
{
    geometry_msgs::PoseStamped pose;
    setOrientationConstraints(move_group);
    pose = setEndEffectorPosoTarget(move_group, x, y, z, 0, 0, 0, false);
    clearConstraints(move_group);
    return pose;
}

geometry_msgs::PoseStamped RubikCubeSolve::setEndEffectorPosoTarget(moveit::planning_interface::MoveGroupInterface& move_group, \
                                                                    double x, double y, double z,
                                                                    double X, double Y, double Z, bool radian=true)
{
    geometry_msgs::PoseStamped poseStamped;
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    // 设置位置
    poseStamped.header.frame_id = move_group.getEndEffectorLink();
    poseStamped.pose.position.x = x;
    poseStamped.pose.position.y = y;
    poseStamped.pose.position.z = z;
    if(!radian)
    {
        X = angle2rad(X);
        Y = angle2rad(Y);
        Z = angle2rad(Z);
    }
    tf2::Quaternion orientation;
    orientation.setEuler(Y, X, Z);
    poseStamped.pose.orientation = tf2::toMsg(orientation);
    TransformToWorldFrame(poseStamped);
    setAndMove(move_group, poseStamped);
    return poseStamped;
}

 double RubikCubeSolve::angle2rad(double& angle)
 {
     angle = (angle/180.0)*180.0;
     return (angle/180)*3.1415926535;
 }

void RubikCubeSolve::setJoint6Value(moveit::planning_interface::MoveGroupInterface& move_group, int angle)
{
    double a = static_cast<double>(angle);
    std::vector<double> joint;
    joint = move_group.getCurrentJointValues();
    a = angle2rad(a);
    joint[5] += a;
    if((joint[5] > 5.0 || joint[5] < -5.0) && (a == 180 || a == -180))
    {
        joint[5] -= 2*a;
    }
    ROS_INFO_STREAM(joint[5]);
    move_group.setJointValueTarget(joint);
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    move_group.plan(my_plan);
    loop_move(move_group);
}

void RubikCubeSolve::getPoseStamped()
{
    move_group0.setNamedTarget("home0");
    move_group1.setNamedTarget("home1");
    move_group0.move();
    move_group1.move();   
    setEulerAngle(move_group0, -1.57, 0, 0);
    setEulerAngle(move_group1, 1.57, 0, 0);
    setEulerAngle(move_group0, 0, -1.57, 0);
    setEulerAngle(move_group1, 0, -1.57, 0);

    // 空間點位3,4
    // poseStmpedS[0] = setEndEffectorPositionTarget(move_group0, 0.3, 0, 0.2);
    // poseStmpedS[1] = setEndEffectorPositionTarget(move_group1, 0.3, 0, 0.2);
    // poseStmpedS[2] = setEndEffectorPositionTarget(move_group0, 0.17, 0, 0);
    // poseStmpedS[3] = setEndEffectorPositionTarget(move_group1, 0.17, 0, 0);
    // poseStmpedS[4] = setEndEffectorPosoTarget(move_group0, 0, 0, 0.2, 0, -0.785, 0);
    // poseStmpedS[5] = setEndEffectorPosoTarget(move_group1, 0, 0, 0.2, 0, -0.785, 0);

    robotPose[0][0] = setEndEffectorPositionTarget(move_group0, data0.pose0[0], data0.pose0[1], data0.pose0[2]);
    robotPose[1][0] = setEndEffectorPositionTarget(move_group1, data1.pose0[0], data1.pose0[1], data1.pose0[2]);
    // 空間點位2
    // 点位1
    ROS_INFO_STREAM("point 1");
    robotPose[0][1] = setEndEffectorPosoTarget(move_group0, data0.pose1[0]-prepare_some_distance, data0.pose1[1]+rubikCubeAdd, data0.pose1[2], -90, 0, 0, false);
    robotPose[1][1] = setEndEffectorPosoTarget(move_group1, data1.pose1[0]-prepare_some_distance, data1.pose1[1]-rubikCubeAdd, data1.pose1[2], 90, 0, 0, false);
    // 点位2
    ROS_INFO_STREAM("point 2");
    robotPose[0][2] = setEndEffectorPosoTarget(move_group0, 0, 0, -2*rubikCubeAdd, 180, 0, 0, false);
    robotPose[1][2] = setEndEffectorPosoTarget(move_group1, 0, 0, -2*rubikCubeAdd, -180, 0, 0, false);
    // 点位3
    ROS_INFO_STREAM("point 3");
    robotPose[0][3] = setEndEffectorPosoTarget(move_group0, 0, rubikCubeAdd, -rubikCubeAdd, -90, 0, 0, false);
    robotPose[1][3] = setEndEffectorPosoTarget(move_group1, 0, -rubikCubeAdd, -rubikCubeAdd, 90, 0, 0, false);
    // 点位4
    ROS_INFO_STREAM("point 4");
    robotPose[0][4] = setEndEffectorPosoTarget(move_group0, 0, 0, -2*rubikCubeAdd, 180, 0, 0, false);
    robotPose[1][4] = setEndEffectorPosoTarget(move_group1, 0, 0, -2*rubikCubeAdd, -180, 0, 0, false);
    // 拧动的位置
    ROS_INFO_STREAM("rotate ");
    robotPose[0][5] = setEndEffectorPosoTarget(move_group0, -rubikCubeAdd, 0, -rubikCubeAdd, -180, 0, 0, false);
    robotPose[1][5] = setEndEffectorPosoTarget(move_group1, -rubikCubeAdd, 0, -rubikCubeAdd, 180, 0, 0, false);
    ROS_INFO_STREAM("center");
    setEndEffectorPosoTarget(move_group0, rubikCubeAdd+prepare_some_distance, 0, 0, 0, 0, 0, false);
    setEndEffectorPosoTarget(move_group1, rubikCubeAdd+prepare_some_distance, 0, 0, 0, 0, 0, false);

    // setEndEffectorPositionTarget(move_group0, this->prepare_some_distance, 0, 0);
    // setEndEffectorPositionTarget(move_group1, this->prepare_some_distance, 0, 0);
    // 空間點位1
    robotPose[0][6] = setEndEffectorPosoTarget(move_group0, data0.pose2[0], data0.pose2[1], data0.pose2[2], data0.pose2[3], data0.pose2[4], data0.pose2[5]);
    robotPose[1][6] = setEndEffectorPosoTarget(move_group1, data1.pose2[0], data1.pose2[1], data1.pose2[2], data1.pose2[3], data1.pose2[4], data1.pose2[5]);


    setEndEffectorPosoTarget(move_group0, -rubikCubeAdd, 0, -rubikCubeAdd, 0, 0, 0);
    setEndEffectorPosoTarget(move_group1, -rubikCubeAdd, 0, -rubikCubeAdd, 0, 0, 0);

    robotPose[0][7] = setEndEffectorPositionTarget(move_group0, -prepare_some_distance, 0, 0);
    robotPose[1][7] = setEndEffectorPositionTarget(move_group1, -prepare_some_distance, 0, 0);
}

void RubikCubeSolve::example()
{
    setAndMove(move_group0, robotPose[0][0]);
    setAndMove(move_group1, robotPose[1][0]);

    setAndMove(move_group0, robotPose[0][1]);
    setAndMove(move_group1, robotPose[1][5]);
    setJoint6Value(move_group1, -90);
    setAndMove(move_group1, robotPose[1][0]);

    setAndMove(move_group0, robotPose[0][6]);
    setAndMove(move_group1, robotPose[1][7]);
    setJoint6Value(move_group1, -90);
    setAndMove(move_group1, robotPose[1][0]);
    setAndMove(move_group0, robotPose[0][1]);
}

int  RubikCubeSolve::writePoseFile()
{
    std::string path;
    YAML::Node doc;

    nh.getParam("/pose03_path", path);
    writePoseOnceFile(path, robotPose[0][0]);

    nh.getParam("/pose021_path", path);
    writePoseOnceFile(path, robotPose[0][1]);

    nh.getParam("/pose022_path", path);
    writePoseOnceFile(path, robotPose[0][2]);

    nh.getParam("/pose023_path", path);
    writePoseOnceFile(path, robotPose[0][3]);

    nh.getParam("/pose024_path", path);
    writePoseOnceFile(path, robotPose[0][4]);

    nh.getParam("/pose025_path", path);
    writePoseOnceFile(path, robotPose[0][5]);

    nh.getParam("/pose011_path", path);
    writePoseOnceFile(path, robotPose[0][6]);

    nh.getParam("/pose012_path", path);
    writePoseOnceFile(path, robotPose[0][7]);
//////////////////////////////////////////////
    nh.getParam("/pose14_path", path);
    writePoseOnceFile(path, robotPose[1][0]);

    nh.getParam("/pose121_path", path);
    writePoseOnceFile(path, robotPose[1][1]);

    nh.getParam("/pose122_path", path);
    writePoseOnceFile(path, robotPose[1][2]);

    nh.getParam("/pose123_path", path);
    writePoseOnceFile(path, robotPose[1][3]);

    nh.getParam("/pose124_path", path);
    writePoseOnceFile(path, robotPose[1][4]);

    nh.getParam("/pose125_path", path);
    writePoseOnceFile(path, robotPose[1][5]);

    nh.getParam("/pose111_path", path);
    writePoseOnceFile(path, robotPose[1][6]);

    nh.getParam("/pose112_path", path);
    writePoseOnceFile(path, robotPose[1][7]);
}

bool RubikCubeSolve::writePoseOnceFile(const std::string& name, const geometry_msgs::PoseStamped& pose)
{
    std::ofstream fout(name, std::ios::out);
    // std::vector<YAML::Node> config;
    YAML::Node config;
    // config.resize(8);
    config["header"]["frame_id"] = pose.header.frame_id;
    config["pose"]["position"]["x"] = pose.pose.position.x;
    config["pose"]["position"]["y"] = pose.pose.position.y;
    config["pose"]["position"]["z"] = pose.pose.position.z;
    config["pose"]["orientation"]["x"] = pose.pose.orientation.x;
    config["pose"]["orientation"]["y"] = pose.pose.orientation.y;
    config["pose"]["orientation"]["z"] = pose.pose.orientation.z;
    config["pose"]["orientation"]["w"] = pose.pose.orientation.w;
    fout << config;
    ROS_INFO_STREAM("write over " << name.c_str());
    fout.close();
}

moveit::planning_interface::MoveItErrorCode RubikCubeSolve::loop_move(moveit::planning_interface::MoveGroupInterface& move_group)
{
    int cnt = 0;
    moveit::planning_interface::MoveItErrorCode code;
    do
    {
        code = move_group.move();
        cnt ++;
    }
    while (ros::ok() && cnt < 10 && code.val == moveit::planning_interface::MoveItErrorCode::TIMED_OUT);
    return code;
}

void RubikCubeSolve::loop()
{
    std::cin.ignore();
    while (ros::ok())
    {
        example();
    }
}