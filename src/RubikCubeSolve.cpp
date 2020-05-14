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
    stop_move = nh.advertiseService("exit_move", &RubikCubeSolve::sotpMoveCallBack, this);
    // ServiceServer = nh.advertiseService("/Rb_runCommand", &RubikCubeSolve::rbRunCommand, this);

    beginSolve = nh.subscribe("beginSolve", 20, &RubikCubeSolve::beginSolve_sub, this);
    rubikCubeSolveData_sub = nh.subscribe("cube_solution", 100, &RubikCubeSolve::rubikCubeSolveDataCallBack, this);

    openGripper_client0 = nh.serviceClient<hirop_msgs::openGripper>("/UR51/openGripper");
    closeGripper_client0 = nh.serviceClient<hirop_msgs::closeGripper>("/UR51/closeGripper");
    openGripper_client1 = nh.serviceClient<hirop_msgs::openGripper>("/UR52/openGripper");
    closeGripper_client1 = nh.serviceClient<hirop_msgs::closeGripper>("/UR52/closeGripper");
    shootClient = nh.serviceClient<cubeParse::TakePhoto>("get_cube");
    receiveSolve = nh.serviceClient<cubeParse::SolveCube>("solve_cube");
    robotPose.resize(ROWS);
    robotPose[0].resize(COLUMNS);
    robotPose[1].resize(COLUMNS);
    photographPose.resize(5);

    double speed;
    nh.getParam("/rubik_cube_solve/speed", speed);

    move_group0.setMaxAccelerationScalingFactor(0.1);
    move_group1.setMaxAccelerationScalingFactor(0.1);
    move_group0.setMaxVelocityScalingFactor(speed);
    move_group1.setMaxVelocityScalingFactor(speed);
    loadPickPose();
    initPose();
    Cstate.isFinish = true;
    isBegingSolve = false;
}
void RubikCubeSolve::beginSolve_sub(const std_msgs::BoolConstPtr& msg)
{
    if(isBegingSolve != true)
    {
        isBegingSolve = true;
    }
}

void RubikCubeSolve::rubikCubeSolveDataCallBack(const std_msgs::Int8MultiArrayConstPtr& msg)
{
    for(int i=0; i < msg->data.size(); i++)
    {
        rubikCubeSolveData.push_back(msg->data[i]);
    }
    Cstate.isFinish = false;
    transformData();
    ROS_INFO_STREAM("catch data");
}

void RubikCubeSolve::transformData()
{
    int angleFlag;
    int angle;
    int face;
    rubikCubeSolvetransformData.resize(rubikCubeSolveData.size());
    for(int i=0; i < rubikCubeSolveData.size(); ++i)
    {
        rubikCubeSolvetransformData[i].resize(2);
        angleFlag =rubikCubeSolveData[i] % 3;
        if(angleFlag == 0)
            rubikCubeSolvetransformData[i][1] = 90;
        else if(angleFlag == 1)
            rubikCubeSolvetransformData[i][1] = 180;
        else 
            rubikCubeSolvetransformData[i][1] = -90;
        if(rubikCubeSolveData[i] >=0 && rubikCubeSolveData[i] <=2)
        {
            face = 6;
        }
        else if (rubikCubeSolveData[i] >=3 && rubikCubeSolveData[i] <=5)
        {
            face = 5;
        }
        else if (rubikCubeSolveData[i] >=6 && rubikCubeSolveData[i] <=8)
        {
            face = 4;
        }
        else if (rubikCubeSolveData[i] >=9 && rubikCubeSolveData[i] <=11)
        {
            face = 3;
        }
        else if (rubikCubeSolveData[i] >=12 && rubikCubeSolveData[i] <=14)
        {
            face = 1;
        }
        else
        {
            face = 2;
        }
        rubikCubeSolvetransformData[i][0] = face;
        ROS_INFO_STREAM("face: " << rubikCubeSolvetransformData[i][0] << "angle: " <<rubikCubeSolvetransformData[i][1]);
    }
}

void RubikCubeSolve::stopMove()
{
    move_group0.stop();
    move_group1.stop();
    system("rosservice call /UR51/set_robot_enable 'enable: false'");
    system("rosservice call /UR52/set_robot_enable 'enable: false'");
    system("rosservice call /UR51/clear_robot_fault '{}'");
    system("rosservice call /UR52/clear_robot_fault '{}'");
    exit(0);
}

void RubikCubeSolve::goPreparePose()
{
    // setJointConstraints(move_group0);
    // setJointConstraints(move_group1);
    setAndMove(move_group1, robotPose[1][0]);
    setAndMove(move_group0, robotPose[0][0]);

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
        ros::Duration(1).sleep();
        setEndEffectorPositionTarget(move_group1, prepare_some_distance, 0, 0);
        closeGripper(move_group1);
        ros::Duration(1).sleep();
        setEndEffectorPositionTarget(move_group1, -prepare_some_distance, 0, 0);
    }
    else
    {
        setEndEffectorPositionTarget(move_group1, prepare_some_distance, 0, 0);
        openGripper(move_group1);
        setEndEffectorPositionTarget(move_group1, -prepare_some_distance, 0, 0);
    }
}

void RubikCubeSolve::photographstep(int posePick, int poseShoot, bool only_pick=false)
{
    // 流程
    photographPickPlace(photographPose[posePick], true);
    // 去到被拍摄位置
    setAndMove(move_group1, photographPose[poseShoot]);
    shoot();
    if(!only_pick)
        photographPickPlace(photographPose[posePick], false);
}

void RubikCubeSolve::photograph()
{
    move_group0.setNamedTarget("home0");
    move_group1.setNamedTarget("home1");
    loop_move(move_group0);
    loop_move(move_group1);
    // 
    // 到达被拍照位置
    setAndMove(move_group0, photographPose[4]);
    photographstep(0, 3);
    photographstep(1, 3);
    // setEndEffectorPositionTarget(move_group1, 0, 0, 0.2);
    move_group1.setNamedTarget("home1");
    move_group1.move();
    setEulerAngle(move_group1, 90, 0, 0, false);
    setEulerAngle(move_group1, 0, -90, 0, false);
    photographstep(2, 3, true);
}


void RubikCubeSolve::placeCube()
{
    move_group0.setNamedTarget("home0");
    move_group1.setNamedTarget("home1");
    loop_move(move_group0);
    loop_move(move_group1);
    setAndMove(getMoveGroup(Adata.captureRobot), photographPose[0]);
    setEndEffectorPositionTarget(getMoveGroup(Adata.captureRobot), prepare_some_distance, 0, 0);
    openGripper(getMoveGroup(Adata.captureRobot));
    setEndEffectorPositionTarget(getMoveGroup(Adata.captureRobot), -prepare_some_distance, 0, 0);
    move_group0.setNamedTarget("home0");
    move_group1.setNamedTarget("home1");
    loop_move(move_group0);
    loop_move(move_group1);
}

void RubikCubeSolve::shoot(int num)
{
    ROS_INFO_STREAM("shoot");
    cubeParse::TakePhoto srv;
    shootClient.call(srv);
    ros::WallDuration(2.0).sleep();

    setJoint6Value(move_group1, 180);
    // setEulerAngle(move_group1, 180, 0, 0, false);
    shootClient.call(srv);
    ros::WallDuration(2.0).sleep();
}

void RubikCubeSolve::fakeInitializationState()
{
    Cstate.captureRobot = 1;
    Cstate.capturePoint = 3;
    Cstate.canRotateFace1 = 5;
    Cstate.canRotateFace2 = 2;
    Adata.captureRobot = Cstate.captureRobot;
    Adata.capturePoint = Cstate.capturePoint;
    Adata.otherRobot = (Cstate.captureRobot + 1)%2;
    geometry_msgs::PoseStamped pose = robotPose[Cstate.captureRobot][Cstate.capturePoint];
    pose.pose.position.y += pow(-1, Cstate.captureRobot)*prepare_some_distance;
    setAndMove(getMoveGroup(Cstate.captureRobot), pose);
    setAndMove(getMoveGroup(Adata.otherRobot), robotPose[Adata.otherRobot][0]);
}

void RubikCubeSolve::setOrientationConstraints(moveit::planning_interface::MoveGroupInterface& move_group, \
                                double x_axis_tolerance,double y_axis_tolerance,double z_axis_tolerance)
{
    // setJointConstraints(move_group);
    static int cnt = 0;
    moveit_msgs::OrientationConstraint ocm;
    ocm.link_name = move_group.getEndEffectorLink();
    ocm.header.frame_id = "world";
    ocm.orientation = move_group.getCurrentPose().pose.orientation;
    // ROS_INFO_STREAM(move_group.getCurrentPose());
    ocm.absolute_x_axis_tolerance = x_axis_tolerance;
    ocm.absolute_y_axis_tolerance = y_axis_tolerance;
    ocm.absolute_z_axis_tolerance = z_axis_tolerance;
    ocm.weight = 1.0;
    moveit_msgs::Constraints con;
    con.orientation_constraints.push_back(ocm);
    move_group.setPathConstraints(con);
    move_group.setPlanningTime(5.0);
    cnt ++;
    ROS_INFO_STREAM(cnt);
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

void RubikCubeSolve::setJointConstraints(moveit::planning_interface::MoveGroupInterface& move_group)
{
    moveit_msgs::JointConstraint jointCon;
    if(move_group.getName() == "arm0")
    {
        jointCon.joint_name = "joint_4";
        jointCon.position = 1.0;
    }
    else
    {
        jointCon.joint_name = "R_joint_4";
        jointCon.position = -1.0;
    }
    jointCon.tolerance_above = 2.0;
    jointCon.tolerance_below = 2.0;
    jointCon.weight = 0.99;
    moveit_msgs::Constraints con;
    con.joint_constraints.push_back(jointCon);
    move_group.setPathConstraints(con);
    move_group.setPlanningTime(2.0);
}

void RubikCubeSolve::setConstraints(moveit::planning_interface::MoveGroupInterface& move_group, \
                                                    double x_axis_tolerance, double y_axis_tolerance,double z_axis_tolerance)
{
    moveit_msgs::Constraints con;
    moveit_msgs::JointConstraint jointCon;
    if(move_group.getName() == "arm0")
    {
        jointCon.joint_name = "joint_4";
        jointCon.position = 1.0;
    }
    else
    {
        jointCon.joint_name = "R_joint_4";
        jointCon.position = -1.0;
    }
    jointCon.tolerance_above = 2.0;
    jointCon.tolerance_below = 2.0;
    jointCon.weight = 1;
    con.joint_constraints.push_back(jointCon);

    moveit_msgs::OrientationConstraint ocm;
    ocm.link_name = move_group.getEndEffectorLink();
    ocm.header.frame_id = "world";
    ocm.orientation = move_group.getCurrentPose().pose.orientation;

    ocm.absolute_x_axis_tolerance = x_axis_tolerance;
    ocm.absolute_y_axis_tolerance = y_axis_tolerance;
    ocm.absolute_z_axis_tolerance = z_axis_tolerance;
    ocm.weight = 1.0;

    con.orientation_constraints.push_back(ocm);
    move_group.setPathConstraints(con);
    move_group.setPlanningTime(5.0);
}

void RubikCubeSolve::clearConstraints(moveit::planning_interface::MoveGroupInterface& move_group)
{
    move_group.clearPathConstraints();
    move_group.clearTrajectoryConstraints();
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
    static int cnt = 0;   
    bool flag;
    nh.getParam("/rubik_cube_solve/is_photo_praph", flag);
    if(cnt == 30 || cnt == 0)
    {
        if(flag)
        {
            if(cnt != 0)
            {
                Cstate.isFinish = true;
                placeCube();
            }
            cnt = 0;
            photograph();
            goPreparePose();
        }
    }
    else
    {
        analyseData(req.face, req.angle);
        action();
    }
    cnt ++;
    rep.isFinish = true;
    return true;
}

bool  RubikCubeSolve::endEffectorPoseCallBack(rubik_cube_solve::end_effector_motion::Request& req, rubik_cube_solve::end_effector_motion::Response& rep)
{
    geometry_msgs::PoseStamped pose;
    pose = setEndEffectorPosoTarget(getMoveGroup(req.group), req.x, req.y, req.z, req.Ex, req.Ey, req.Ez, false, false);
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

void RubikCubeSolve::spin()
{
    while (ros::ok())
    {
        if(isBegingSolve ==  true)
        {
            ROS_INFO("start");
            photograph();
            ros::Duration(1.0).sleep();
            cubeParse::SolveCube srv;
            // receiveSolve.call(srv);
            goPreparePose();
            for(int i=0; i<rubikCubeSolvetransformData.size(); ++i)
            {
                analyseData(rubikCubeSolvetransformData[i][0], rubikCubeSolvetransformData[i][1]);
                action();
            }
            std::vector<std::vector<int> >().swap(rubikCubeSolvetransformData);
            Cstate.isFinish = true;
            placeCube();
            isBegingSolve = false;
        }
    }
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
    setOrientationConstraints(capture_move_group, 0.1, 0.1, 0.1);
    setEndEffectorPositionTarget(capture_move_group, prepare_some_distance, 0, 0);
    clearConstraints(capture_move_group);
    closeGripper(capture_move_group);

    openGripper(rotate_move_group);
    setOrientationConstraints(rotate_move_group, 0.1, 0.1, 0.1);
    setEndEffectorPositionTarget(rotate_move_group, -prepare_some_distance, 0, 0);
    clearConstraints(rotate_move_group);
    setAndMove(rotate_move_group, robotPose[Adata.otherRobot][0]);
}

bool RubikCubeSolve::step(moveit::planning_interface::MoveGroupInterface& capture_move_group,\
                moveit::planning_interface::MoveGroupInterface& rotate_move_group,\
                geometry_msgs::PoseStamped pose, int space_point, int angle)
{
    if(space_point == UP)
    {
        setAndMove(capture_move_group, robotPose[Adata.captureRobot][3]);
        setAndMove(capture_move_group, robotPose[Adata.captureRobot][6]);

        rotateCube(rotate_move_group, robotPose[Adata.otherRobot][7], angle);
    }
    else
    {
        rotateCube(rotate_move_group, robotPose[Adata.otherRobot][5], angle);
    }
    // 拧魔方的回原位
    setOrientationConstraints(rotate_move_group, 0.1, 0.1, 0.1);
    setEndEffectorPositionTarget(rotate_move_group, -prepare_some_distance, 0, 0);
    clearConstraints(rotate_move_group);
    setAndMove(rotate_move_group, robotPose[Adata.otherRobot][0]);
    // 抓住魔方的回原位
    pose.pose.position.y += pow(-1, Adata.captureRobot)*prepare_some_distance;
    if(Adata.space == UP)
        setAndMove(capture_move_group, robotPose[Adata.captureRobot][3]);
    setAndMove(capture_move_group, pose);
}

bool RubikCubeSolve::rotateCube(moveit::planning_interface::MoveGroupInterface& rotate_move_group, 
                                geometry_msgs::PoseStamped pose, int angle)
{
    openGripper(rotate_move_group);
    setAndMove(rotate_move_group, pose);
    setOrientationConstraints(rotate_move_group, 0.1, 0.1, 0.1);
    setEndEffectorPositionTarget(rotate_move_group, prepare_some_distance, 0, 0);
    clearConstraints(rotate_move_group);
    closeGripper(rotate_move_group);

    rotate_move_group.clearPathConstraints();
    rotate_move_group.clearTrajectoryConstraints();
    setJoint6Value(rotate_move_group, angle);
}

bool RubikCubeSolve::openGripper(moveit::planning_interface::MoveGroupInterface& move_group)
{
    bool flag;
    hirop_msgs::openGripper srv;
    if(move_group.getName() == "arm0"){
        flag = openGripper_client0.call(srv);
        ROS_INFO("arm0 openGripper ");
    }
    else
    {
        ROS_INFO("arm1 openGripper ");
        flag = openGripper_client1.call(srv);
    }
    return flag;
}
bool RubikCubeSolve::closeGripper(moveit::planning_interface::MoveGroupInterface& move_group)
{
    bool flag;
    hirop_msgs::closeGripper srv;
    if(move_group.getName() == "arm0"){
        ROS_INFO("arm0 closeGripper ");
        flag = closeGripper_client0.call(srv);
    }
    else
    {
        ROS_INFO("arm1 closeGripper ");
        flag = closeGripper_client1.call(srv);
    }
    return flag;
}

void RubikCubeSolve::getPrepareSomeDistanceRobotPose()
{
    const double cos45 = 0.7071067;
    for(int i=0; i<ROWS; ++i)
        for(int j=0; j<COLUMNS; ++j)
        {
            if(j==7)
            {
                robotPose[i][j].pose.position.z -= 0.7071067*prepare_some_distance;
                robotPose[i][j].pose.position.y -= pow(-1, i)*prepare_some_distance;
                continue;
            }
            getPrepareSomeDistance(robotPose, i, j);
        }
}

void RubikCubeSolve::initPose()
{
    bool is_read_pose;
    nh.getParam("/rubik_cube_solve/is_load_pose", is_read_pose);
    if(is_read_pose)
    {
        this->loadRobotPoses();
    }
    else
    {
        loadPoseData();
        this->getPoseStamped();
        writePoseFile();
    }
    getPrepareSomeDistanceRobotPose();
}

inline void RubikCubeSolve::getPrepareSomeDistance(std::vector<std::vector<geometry_msgs::PoseStamped> >& pose, int row, int column)
{
    if(column != 0 && column != 6)
        pose[row][column].pose.position.y -= pow(-1, row)*prepare_some_distance;
}

inline void RubikCubeSolve::loadRobotPose(std::string paramName, int row, int column)
{
    YAML::Node doc;
    std::string path;

    nh.getParam(paramName, path);
    doc = YAML::LoadFile(path);
    addData(robotPose[row][column], doc);
}

void RubikCubeSolve::loadRobotPoses()
{
    loadRobotPose("/pose03_path", 0, 0);
    loadRobotPose("/pose021_path", 0, 1);
    loadRobotPose("/pose022_path", 0, 2);
    loadRobotPose("/pose023_path", 0, 3);
    loadRobotPose("/pose024_path", 0, 4);
    loadRobotPose("/pose025_path", 0, 5);
    loadRobotPose("/pose011_path", 0, 6);
    loadRobotPose("/pose012_path", 0, 7);
//////////////////////////////////////////////
    loadRobotPose("/pose14_path", 1, 0);
    loadRobotPose("/pose121_path", 1, 1);
    loadRobotPose("/pose122_path", 1, 2);
    loadRobotPose("/pose123_path", 1, 3);
    loadRobotPose("/pose124_path", 1, 4);
    loadRobotPose("/pose125_path", 1, 5);
    loadRobotPose("/pose111_path", 1, 6);
    loadRobotPose("/pose112_path", 1, 7);
}

void RubikCubeSolve::loadPickPose()
{
    std::string path;
    YAML::Node doc;
    std::vector<std::string> pickPoseParam = {"/pick0_path", "/pick1_path", "/pick2_path",\
                                            "/pick3_path", "/pick4_path"};
    for(int i=0; i < pickPoseParam.size(); ++i)
    {
        nh.getParam(pickPoseParam[i], path);
        doc = YAML::LoadFile(path);
        addData(photographPose[i], doc);
        if(i < 2)
        {
            photographPose[i].pose.position.z += prepare_some_distance;
        }
        else if(i == 2)
        {
            photographPose[i].pose.position.y += prepare_some_distance;
        }
    }
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

bool RubikCubeSolve::transformFrame(geometry_msgs::PoseStamped& poseStamped, std::string frame_id="world")
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
            listener.transformPose(frame_id, otherFramePose[0], worldFramePose[0]);
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
    // move_group.setStartStateToCurrentState();
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    move_group.setPoseTarget(poseStamped);
    this->moveGroupPlanAndMove(move_group, my_plan);
}

moveit::planning_interface::MoveItErrorCode RubikCubeSolve::moveGroupPlanAndMove(moveit::planning_interface::MoveGroupInterface& move_group, \
                                                                moveit::planning_interface::MoveGroupInterface::Plan& my_plan)
{
    while (ros::ok())
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
    return setEndEffectorPosoTarget(move_group, 0, 0, 0, x, y, z, radian, false);
}

geometry_msgs::PoseStamped RubikCubeSolve::setEndEffectorPositionTarget(moveit::planning_interface::MoveGroupInterface& move_group, double x, double y, double z)
{
    geometry_msgs::PoseStamped pose;
    pose = setEndEffectorPosoTarget(move_group, x, y, z, 0, 0, 0, false, false);
    return pose;
}

geometry_msgs::PoseStamped RubikCubeSolve::setEndEffectorPosoTarget(moveit::planning_interface::MoveGroupInterface& move_group, \
                                                                    double x, double y, double z,
                                                                    double X, double Y, double Z, bool radian=true, bool only_get_pose=false)
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
    transformFrame(poseStamped);
    if(!only_get_pose)
        setAndMove(move_group, poseStamped);
    return poseStamped;
}

 double RubikCubeSolve::angle2rad(double& angle)
 {

    return (angle/180)*3.1415926535;
 }

void RubikCubeSolve::setJoint6Value(moveit::planning_interface::MoveGroupInterface& move_group, int angle)
{
    double a = static_cast<double>(angle);
    std::vector<double> joint;
    joint = move_group.getCurrentJointValues();
    a = angle2rad(a);
    joint[5] += a;
    system("rosservice call /remove_objec 'data: true'");
    if((joint[5] > 5.0 || joint[5] < -5.0) && (angle == 180 || angle == -180))
    {
        ROS_INFO_STREAM("joint_6:" << joint[5]);
        joint[5] -= 2*a;
    }
    else if((joint[5] > 5.0 || joint[5] < -5.0) && (angle == 90 || angle == -90))
    {
        joint[5] -= 4*a;
    }

    ROS_INFO_STREAM("joint_6:" << joint[5]);
    move_group.clearPoseTarget();
    move_group.setJointValueTarget(joint);
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    // moveGroupPlanAndMove(move_group, my_plan);
    while (ros::ok())
    {
        if(move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS)
        {
            move_group.execute(my_plan);
            break;
        }
    }
    system("rosservice call /remove_objec 'data: false'");
}

void RubikCubeSolve::getPoseStamped()
{
    move_group0.setNamedTarget("home0");
    move_group1.setNamedTarget("home1");
    loop_move(move_group0);
    loop_move(move_group1);
    setEulerAngle(move_group0, -1.57, 0, 0);
    setEulerAngle(move_group1, 1.57, 0, 0);
    setEulerAngle(move_group0, 0, -1.57, 0);
    setEulerAngle(move_group1, 0, -1.57, 0);

    // 空間點位3,4

    robotPose[0][0] = setEndEffectorPositionTarget(move_group0, data0.pose0[0], data0.pose0[1], data0.pose0[2]);
    robotPose[1][0] = setEndEffectorPositionTarget(move_group1, data1.pose0[0], data1.pose0[1], data1.pose0[2]);
    // 空間點位2
    // 点位1
    ROS_INFO_STREAM("point 1");

    robotPose[0][1] = setEndEffectorPosoTarget(move_group0, data0.pose1[0], data0.pose1[1]+rubikCubeAdd, data0.pose1[2], -90, 0, 0, false);
    robotPose[1][1] = setEndEffectorPosoTarget(move_group1, data1.pose1[0], data1.pose1[1]-rubikCubeAdd, data1.pose1[2], 90, 0, 0, false);
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

    setEndEffectorPosoTarget(move_group0, rubikCubeAdd, 0, 0, 0, 0, 0, false);
    setEndEffectorPosoTarget(move_group1, rubikCubeAdd, 0, 0, 0, 0, 0, false);

    // 空間點位1
    robotPose[0][6] = setEndEffectorPosoTarget(move_group0, data0.pose2[0], data0.pose2[1], data0.pose2[2], data0.pose2[3], data0.pose2[4], data0.pose2[5]);
    robotPose[1][6] = setEndEffectorPosoTarget(move_group1, data1.pose2[0], data1.pose2[1], data1.pose2[2], data1.pose2[3], data1.pose2[4], data1.pose2[5]);

    robotPose[0][7] = setEndEffectorPosoTarget(move_group0, -rubikCubeAdd, 0, -rubikCubeAdd, 0, 0, 0);
    robotPose[1][7] = setEndEffectorPosoTarget(move_group1, -rubikCubeAdd, 0, -rubikCubeAdd, 0, 0, 0);
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
