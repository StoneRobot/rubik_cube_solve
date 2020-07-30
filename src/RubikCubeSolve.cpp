#include <rubik_cube_solve/RubikCubeSolve.h>
#include <math.h> 
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
    goToPose = nh.advertiseService("go_to_pose", &RubikCubeSolve::goToPoseServer, this);
    beginSolve = nh.advertiseService("/MagicStepRunCommand", &RubikCubeSolve::rbRunCommand, this);
    placeCubeServer = nh.advertiseService("/placeMagicCube", &RubikCubeSolve::placeCubeCallback, this);

    magicMoveToPoint = nh.advertiseService("/magic_move_to_point", &RubikCubeSolve::magicMoveToPointCallback, this);
    magicStepMove = nh.advertiseService("/magic_step_move", &RubikCubeSolve::magicStepMoveCallback, this);
    magicRecordPose = nh.advertiseService("/magic_recordPose", &RubikCubeSolve::magicRecordPoseCallback, this);
    

    progressPub = nh.advertise<std_msgs::Int8MultiArray>("progress_rbSolveMagic", 10);

    stopMoveSub = nh.subscribe("/stop_move", 1, &RubikCubeSolve::sotpMoveCallback, this);
    rubikCubeSolveData_sub = nh.subscribe("cube_solution", 100, &RubikCubeSolve::rubikCubeSolveDataCallBack, this);

    openGripper_client0 = nh.serviceClient<hirop_msgs::openGripper>("/UR51/openGripper");
    closeGripper_client0 = nh.serviceClient<hirop_msgs::closeGripper>("/UR51/closeGripper");
    openGripper_client1 = nh.serviceClient<hirop_msgs::openGripper>("/UR52/openGripper");
    closeGripper_client1 = nh.serviceClient<hirop_msgs::closeGripper>("/UR52/closeGripper");
    shootClient = nh.serviceClient<cubeParse::TakePhoto>("get_cube");
    receiveSolve = nh.serviceClient<cubeParse::SolveCube>("solve_cube");

    initMotionClient();
    

    robotPose.resize(ROWS);
    robotPose[0].resize(COLUMNS);
    robotPose[1].resize(COLUMNS);
    
    setJointConstraints(move_group0);
    setJointConstraints(move_group1);
    ROS_INFO_STREAM(move_group0.getPathConstraints());
    ROS_INFO_STREAM(move_group1.getPathConstraints());

    double speed;
    nh.getParam("/rubik_cube_solve/speed", speed);
    move_group0.setMaxVelocityScalingFactor(speed);
    move_group1.setMaxVelocityScalingFactor(speed);
    move_group0.setGoalPositionTolerance(0.0001);
    move_group1.setGoalPositionTolerance(0.0001);
    loadPickPose();
    initPose();
    Cstate.isFinish = false;
    isBegingSolve = false;
    isStop = false;
}

/****/
void RubikCubeSolve::initMotionClient()
{
    l_moveToSiglePose_client = nh.serviceClient<hirop_msgs::moveToSiglePose>("/hsr_left/moveToSiglePose");
    r_moveToSiglePose_client = nh.serviceClient<hirop_msgs::moveToSiglePose>("/hsr_left/moveToSiglePose");

    l_moveToMultiPose_client = nh.serviceClient<hirop_msgs::moveToMultiPose>("/hsr_left/moveToMultiPose");
    r_moveToMultiPose_client = nh.serviceClient<hirop_msgs::moveToMultiPose>("/hsr_left/moveToMultiPose");

    l_moveLine_client = nh.serviceClient<hirop_msgs::moveLine>("/hsr_left/moveLine");
    r_moveLine_client = nh.serviceClient<hirop_msgs::moveLine>("/hsr_left/moveLine");

    l_SigleAixs_client = nh.serviceClient<hirop_msgs::moveSigleAixs>("/hsr_left/SigleAixs");
    r_SigleAixs_client = nh.serviceClient<hirop_msgs::moveSigleAixs>("/hsr_left/SigleAixs");
}

std::vector<double> RubikCubeSolve::getRobotState(moveit::planning_interface::MoveGroupInterface& move_group, geometry_msgs::PoseStamped& poseStamped)
{
    setStartState(move_group);
    unsigned int attempts = 10;
    double timeout = 0.1;
    robotModel = move_group.getRobotModel();
    robotState = move_group.getCurrentState();
    jointModelGroup = const_cast<robot_state::JointModelGroup*>(robotModel->getJointModelGroup(move_group.getName()));
    std::vector<double> joints;
    for(int i=0; i<2; i++)
    {
        bool flag;
        flag = robotState->setFromIK(jointModelGroup, poseStamped.pose, attempts, timeout);
        if(flag)
        {
            ROS_INFO("IK SUCCEED");
            robotState->copyJointGroupVelocities(jointModelGroup, joints);
            return joints;
        }
    }
    ROS_INFO("IK FAILED");
    return joints;
}

bool RubikCubeSolve::setAndMoveClient(moveit::planning_interface::MoveGroupInterface& move_group, \
                                        geometry_msgs::PoseStamped& poseStamped)
{
    std::vector<double> joints;
    joints = getRobotState(move_group, poseStamped);
    if(joints.empty())
        return false;
    hirop_msgs::moveToSiglePose srv;
    srv.request.pose_joints_angle.joints_angle.data = joints;
    if(move_group.getName() == "arm0")
    {
        if(l_moveToSiglePose_client.call(srv))
        {
            ROS_INFO(srv.response.info.c_str());
            return srv.response.is_success;
        }
    }
    else
    {
        if(r_moveToSiglePose_client.call(srv))
        {
            ROS_INFO(srv.response.info.c_str());
            return srv.response.is_success;
        }
    }
    return false;
}

bool RubikCubeSolve::robotMoveCartesianUnitClient(moveit::planning_interface::MoveGroupInterface &group, double x, double y, double z)
{
    hirop_msgs::moveLine srv;
    srv.request.Cartesian_x = x;
    srv.request.Cartesian_y = y;
    srv.request.Cartesian_z = z;
    if(group.getName() == "arm0")
    {
        if(l_moveLine_client.call(srv))
        {
            ROS_INFO(srv.response.info.c_str());
            return srv.response.is_success;
        }
    }
    else
    {
        if(r_moveLine_client.call(srv))
        {
            ROS_INFO(srv.response.info.c_str());
            return srv.response.is_success;
        }
    }
    return false;
}

bool RubikCubeSolve::setJoint6ValueClient(moveit::planning_interface::MoveGroupInterface& rotate_move_group, int angle)
{
    setStartState(rotate_move_group);
    std::vector<double> joint = rotate_move_group.getCurrentJointValues();
    double a = static_cast<double>(angle);
    a = angle2rad(a);
    joint[5] += a;

    if((joint[5] > 5.0 || joint[5] < -5.0) && (angle == 180 || angle == -180))
    {
        ROS_INFO_STREAM("joint_6:" << joint[5]);
        joint[5] -= 2*a;
    }
    else if((joint[5] > 5.0 || joint[5] < -5.0) && (angle == 90 || angle == -90))
    {
        joint[5] -= 4*a;
    }
    hirop_msgs::moveSigleAixs srv;
    srv.request.angle = joint[5];
    srv.request.index_axis = 5;
    if(rotate_move_group.getName() == "arm0")
    {
        if(l_moveLine_client.call(srv))
        {
            ROS_INFO(srv.response.info.c_str());
            return srv.response.is_success;
        }
    }
    else
    {
        if(r_moveLine_client.call(srv))
        {
            ROS_INFO(srv.response.info.c_str());
            return srv.response.is_success;
        }
    }
    return false;
}


/****/

bool RubikCubeSolve::magicMoveToPointCallback(rb_msgAndSrv::rb_ArrayAndBool::Request& req, rb_msgAndSrv::rb_ArrayAndBool::Response& rep)
{
    recordPointData.model = req.data[0];
    recordPointData.stepNum = req.data[1];
    ROS_INFO_STREAM(recordPointData.model << recordPointData.stepNum);
    moveToPose();
    rep.respond = true;
    return true;
}

bool RubikCubeSolve::magicStepMoveCallback(std_srvs::Empty::Request& req, std_srvs::Empty::Response& rep)
{
    Cartesian();
    return true;
}

bool RubikCubeSolve::magicRecordPoseCallback(std_srvs::Empty::Request& req, std_srvs::Empty::Response& rep)
{
    updataPointData();
    return true;
}

bool RubikCubeSolve::placeCubeCallback(rb_msgAndSrv::rb_ArrayAndBool::Request& req, rb_msgAndSrv::rb_ArrayAndBool::Response& rep)
{
    stopMove();
    placeCubeRobot = req.data[0];
    ROS_INFO_STREAM(placeCubeRobot);
    if(isBegingSolve || isTest)
        isPlaceCube = true;
    else
    {
        placeCube();
    }
    return true;
}


bool RubikCubeSolve::goToPoseServer(rubik_cube_solve::recordPoseStamped::Request& req, rubik_cube_solve::recordPoseStamped::Response& rep)
{
    std::string path;
    nh.getParam("/rubik_cube_solve/record_pose_path", path);
    path += "/recordPose/" + req.PoseName;
    YAML::Node doc;
    doc = YAML::LoadFile(path);
    if(req.isJointSpace)
    {
        ROS_DEBUG("record joint space in development ...");
        rep.isFinish = false;
    }
    else
    {
        geometry_msgs::PoseStamped pose;

        addData(pose, doc);
        ROS_INFO_STREAM(pose);
        setAndMoveClient(getMoveGroup(req.robot), pose);
    }
    return true;
}

bool RubikCubeSolve::rbRunCommand(rb_msgAndSrv::rb_ArrayAndBool::Request& req, rb_msgAndSrv::rb_ArrayAndBool::Response& rep)
{
    nh.setParam("/isRuning_solveMagic", true);
    if(isBegingSolve != true)
    {
        isBegingSolve = true;
    }
    runModel = req.data[0];
    isStop = false;
    spinOnce();
    rep.respond = true;
    isStop = false;
    return true;
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
        ROS_INFO_STREAM("face: " << rubikCubeSolvetransformData[i][0] << ", angle: " <<rubikCubeSolvetransformData[i][1]);
    }
    std::vector<int>().swap(rubikCubeSolveData);
}

void RubikCubeSolve::stopMove()
{
    move_group0.stop();
    move_group1.stop();
    isStop = true;
}

void RubikCubeSolve::goPreparePose()
{
    setAndMoveClient(move_group1, robotPose[1][0]);
    setAndMoveClient(move_group0, robotPose[0][0]);

    bool fake;
    nh.getParam("/rubik_cube_solve/fakeInitializationState",fake);
    if(fake)
        InitializationStateAction();

}


void RubikCubeSolve::photographPickPlace(int robotNum, geometry_msgs::PoseStamped& pose, bool isPick, int pre_grasp_approach[], int post_grasp_retreat[])
{
    setAndMoveClient(getMoveGroup(robotNum), pose);
    if(isPick)
    { 
        //Z AXIS
        ROS_INFO("openGripper --- 1");
        openGripper(getMoveGroup(robotNum));
        ros::Duration(1).sleep();
        // 寸进的过程 前进1
        robotMoveCartesianUnitClient(getMoveGroup(robotNum), pre_grasp_approach[0]*prepare_some_distance, \
                                pre_grasp_approach[1]*prepare_some_distance, pre_grasp_approach[2]*prepare_some_distance);

        ROS_INFO("closeGripper --- 2");
        closeGripper(getMoveGroup(robotNum));
        // 寸进的过程 后退
        robotMoveCartesianUnitClient(getMoveGroup(robotNum), post_grasp_retreat[0]*prepare_some_distance, \
                                post_grasp_retreat[1]*prepare_some_distance, post_grasp_retreat[2]*prepare_some_distance);
        ROS_INFO("PICK UP  THE CUBE Z");
    }
    else
    {
        //Y AXIS
        robotMoveCartesianUnitClient(getMoveGroup(robotNum), pre_grasp_approach[0]*prepare_some_distance, \
                                pre_grasp_approach[1]*prepare_some_distance, pre_grasp_approach[2]*prepare_some_distance);
        openGripper(getMoveGroup(robotNum));
        robotMoveCartesianUnitClient(getMoveGroup(robotNum), post_grasp_retreat[0]*prepare_some_distance, \
                                post_grasp_retreat[1]*prepare_some_distance, post_grasp_retreat[2]*prepare_some_distance);
        ROS_INFO("PLACE UP  THE CUBE Y");
    }
}

void RubikCubeSolve::photographstepDoublePhoto(int photoNum, int capturePose, int talkPose)
{
    if(!isStop)
    {
        setAndMoveClient(getMoveGroup(Adata.captureRobot), robotPose[Adata.captureRobot][3]);
        
        setAndMoveClient(getMoveGroup(Adata.captureRobot), photographPose[capturePose]);

        setAndMoveClient(getMoveGroup(Adata.otherRobot), photographPose[talkPose]);

        shoot(photoNum);
        setJoint6ValueClient(getMoveGroup(Adata.captureRobot), 180);
        shoot(++photoNum);
        setAndMoveClient(getMoveGroup(Adata.otherRobot), robotPose[Adata.otherRobot][0]);

        setAndMoveClient(getMoveGroup(Adata.captureRobot), robotPose[Adata.captureRobot][3]);
        geometry_msgs::PoseStamped pose = robotPose[Adata.captureRobot][Adata.capturePoint];
        pose.pose.position.y += pow(-1, Adata.captureRobot)*prepare_some_distance;
        setAndMoveClient(getMoveGroup(Adata.captureRobot), pose);
    }
}

void RubikCubeSolve::shoot(int num)
{
    if(!isStop)
    {
        ROS_INFO_STREAM("shoot" << num);
        cubeParse::TakePhoto srv;
        srv.request.photoNum = num;
        shootClient.call(srv);
        ros::WallDuration(2.0).sleep();
    }
}

// 拍照点位
void RubikCubeSolve::photograph()
{
    backHome(1);
    // 抓起魔方
    // PosepickPose0.yaml
    pickCube();
    goPreparePose();
    // PosepickPose1.yaml
    setAndMoveClient(move_group1, photographPose[1]);
    // PosepickPose2.yaml
    setAndMoveClient(move_group0, photographPose[2]);
    shoot(0);

    setAndMoveClient(move_group1, robotPose[1][0]);
    setAndMoveClient(move_group0, robotPose[0][0]);

    // robot1 PosepickPose3.yaml; robot0 PosepickPose4.yaml
    photographstepDoublePhoto(1, 3, 4);

    analyseData(3, 0);
    swop(getMoveGroup(Adata.captureRobot), getMoveGroup(Adata.otherRobot), robotPose[Adata.captureRobot][Adata.capturePoint]);

    // robot0 PosepickPose5.yaml
    setAndMoveClient(move_group0, photographPose[5]);
    // robot1 PosepickPose6.yaml
    setAndMoveClient(move_group1, photographPose[6]);
    shoot(3);

    setAndMoveClient(move_group1, robotPose[1][0]);
    setAndMoveClient(move_group0, robotPose[0][0]);
    // robot0 PosepickPose3.yam7; robot1 PosepickPose8.yaml
    photographstepDoublePhoto(4, 7, 8);
}

bool RubikCubeSolve::setRobotEnable()
{
    system("rosrun rubik_cube_solve set_robot_enable_false.sh");
    system("rosrun rubik_cube_solve set_robot_enable_true.sh");
    return true;
}


void RubikCubeSolve::placeCube()
{
    isStop = false;
    int robotNum = placeCubeRobot;
    backHome(0);
    backHome(1);
    tf2::Quaternion orientation;
    geometry_msgs::PoseStamped placeCubePose;
    ROS_INFO_STREAM("robotNum: " << robotNum);
    if(robotNum == 0)
    {
        placeCubePose = photographPose[9];
    }
    else
    {
        placeCubePose = photographPose[0];
    }
    setAndMoveClient(getMoveGroup(robotNum), placeCubePose);
    robotMoveCartesianUnitClient(getMoveGroup(robotNum), 0, 0, -prepare_some_distance);
    openGripper(getMoveGroup(robotNum));
    robotMoveCartesianUnitClient(getMoveGroup(robotNum), 0, 0, prepare_some_distance);
    backHome(robotNum);
    isPlaceCube = false;
    nh.setParam("/isRuning_solveMagic", false);
}



void RubikCubeSolve::InitializationState()
{
    Cstate.captureRobot = 1;
    Cstate.capturePoint = 3;
    Cstate.canRotateFace1 = 5;
    Cstate.canRotateFace2 = 2; 
    Adata.captureRobot = Cstate.captureRobot;
    Adata.capturePoint = Cstate.capturePoint;
    Adata.otherRobot = (Cstate.captureRobot + 1)%2;
}

bool RubikCubeSolve::InitializationStateAction()
{
    geometry_msgs::PoseStamped pose = robotPose[Cstate.captureRobot][Cstate.capturePoint];
    pose.pose.position.y += pow(-1, Cstate.captureRobot)*prepare_some_distance;
    setAndMoveClient(getMoveGroup(Cstate.captureRobot), pose);
    setAndMoveClient(getMoveGroup(Adata.otherRobot), robotPose[Adata.otherRobot][0]);
}

// TODO
void RubikCubeSolve::setOrientationConstraints(moveit::planning_interface::MoveGroupInterface& move_group, \
                                double x_axis_tolerance,double y_axis_tolerance,double z_axis_tolerance)
{
    static int cnt = 0;
    moveit_msgs::OrientationConstraint ocm;
    ocm.link_name = move_group.getEndEffectorLink();
    ocm.header.frame_id = "world";
    ocm.orientation = move_group.getCurrentPose().pose.orientation;
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
    moveit_msgs::Constraints con;
    // 4 joint
    if(move_group.getName() == "arm0")
    {
        jointCon.joint_name = "joint_4";
        jointCon.position = 1.0;
        jointCon.tolerance_above = 2;
        jointCon.tolerance_below = 1.5;
    }
    else
    {
        jointCon.joint_name = "R_joint_4";
        jointCon.position = -1.0;
        jointCon.tolerance_above = 1.5;
        jointCon.tolerance_below = 2.0;
    }
    jointCon.weight = 1;

    con.joint_constraints.push_back(jointCon);
    move_group.setPathConstraints(con);
    move_group.setPlanningTime(1.0);
}

void RubikCubeSolve::setJointAllConstraints(moveit::planning_interface::MoveGroupInterface& move_group)
{
    moveit_msgs::JointConstraint jointCon;
    moveit_msgs::Constraints con;
    // set 6 joint
    std::vector<double> joint = move_group.getCurrentJointValues();
    if(move_group.getName() == "arm0")
    {
        jointCon.joint_name = "joint_6";
    }
    else
    {
        jointCon.joint_name = "R_joint_6";
    }
    jointCon.position = joint[5];
    jointCon.tolerance_above = 4.8844;
    jointCon.tolerance_below = 4.8844;
    jointCon.weight = 0.9;
    con.joint_constraints.push_back(jointCon);

    // 4 joint
    if(move_group.getName() == "arm0")
    {
        jointCon.joint_name = "joint_4";
        jointCon.position = 1.0;
        jointCon.tolerance_above = 2;
        jointCon.tolerance_below = 1.5;
    }
    else
    {
        jointCon.joint_name = "R_joint_4";
        jointCon.position = -1.0;
        jointCon.tolerance_above = 1.5;
        jointCon.tolerance_below = 2.0;
    }
    jointCon.weight = 1;

    con.joint_constraints.push_back(jointCon);
    move_group.setPathConstraints(con);
    move_group.setPlanningTime(1.0);
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
    isTest = true;
    static int cnt = 0;
    int flag;
    nh.getParam("/rubik_cube_solve/test", flag);
    if(req.face == 0)
    {
        cnt ++;
        if(flag == 0)
        {
            // 測試拍照
            ROS_INFO_STREAM("test 0");
            photograph();
        }
        else if(flag == 1)
        {
            // 去到預備動作
            ROS_INFO_STREAM("test 1");
            InitializationState();
            goPreparePose();
        }
        else if(flag ==2)
        {
            // 測試拿起魔方的動作
            ROS_INFO_STREAM("test 2");
            backHome(1);
            setEulerAngle(move_group1, 90, 0, 0, false);
            setEulerAngle(move_group1, 0, -90, 0, false);
            int pre_grasp_approach[3] = {0, -1, 0};
            int post_grasp_retreat[3] = {0, 0, 1};
            photographPickPlace(1, photographPose[0], true, pre_grasp_approach, post_grasp_retreat);
            InitializationState();
            goPreparePose();
        }
        else if(flag == 3)
        {
            // 测试魔方解算数据
            backHome(1);
            pickCube();
            cubeParse::SolveCube srv;
            receiveSolve.call(srv);
            goPreparePose();
            int cnt = 1;
            for(auto it: rubikCubeSolvetransformData)
            {
                ROS_INFO_STREAM("step: " << cnt << " " << it[0] << " " << it[1]);
                cnt ++;
            }
            cnt = 1;
            for(int i=0; i<rubikCubeSolvetransformData.size() && ros::ok() && !isStop; ++i)
            {
                ROS_INFO("step: %d, count: %d",cnt, rubikCubeSolvetransformData.size());
                cnt ++;
                analyseData(rubikCubeSolvetransformData[i][0], rubikCubeSolvetransformData[i][1]);
                action();
            }
            std::vector<std::vector<int> >().swap(rubikCubeSolvetransformData);
            Cstate.isFinish = true;
            placeCube();
            isBegingSolve = false;
        }
        else if(flag == 4)
        {
            // 測試機器人1的精度
            ROS_INFO_STREAM("test 4");
            backHome(1);
            setAndMoveClient(move_group1, photographPose[0]);
            openGripper(move_group1);
            robotMoveCartesianUnitClient(move_group1, 0, 0, -prepare_some_distance);
        }
        else if (flag == 5)
        {
            // 放置魔方
            ROS_INFO_STREAM("test 5");
            InitializationState();
            goPreparePose();
            placeCube();
        }
        else if(flag == 6)
        {
            // 測試拿起魔方的動作
        }
        else if(flag == 7)
        {
            // 測試拿起魔方的動作
        }
        else if(flag == 8)
        {
            // 測試機器人1的精度
            ROS_INFO_STREAM("test 8");
            backHome(1);
            pickCube();
            goPreparePose();
        }
        else if(flag  == 9)
        {
            ROS_INFO_STREAM("test 9");
            photograph();
        }
    }
    else
    {
        if(cnt == 0)
        {
            InitializationState();
            goPreparePose();
        }
        analyseData(req.face, req.angle);
        action();
    }
    rep.isFinish = true;
    if(isPlaceCube)
    {
        placeCube();
    }
    isTest = false;
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
    recordPose(req.robot, req.PoseName, req.isJointSpace);
    rep.isFinish = true;
    return true;
}

bool RubikCubeSolve::setStartState(moveit::planning_interface::MoveGroupInterface& group)
{
    std::vector<double> joint = group.getCurrentJointValues();
    moveit_msgs::RobotState r;
    r.joint_state.position = joint;
    group.setStartState(r);
    group.setStartStateToCurrentState();
}

bool RubikCubeSolve::recordPose(int robotNum, std::string name, bool isJointSpce=false)
{
    setStartState(getMoveGroup(robotNum));
    std::string path;
    nh.getParam("/rubik_cube_solve/record_pose_path", path);
    // $(find rubik_cube_solve)/recordPose/*.yaml
    path += "/recordPose/" + name + ".yaml";
    if(isJointSpce)
    {
        ROS_DEBUG("record joint space in development ...");
    }
    else
    {
        geometry_msgs::PoseStamped pose;
        getMoveGroup(robotNum).getCurrentPose();
        pose = getMoveGroup(robotNum).getCurrentPose();
        ROS_INFO_STREAM(pose);
        writePoseOnceFile(path, pose);
    }
    return true;
}

void RubikCubeSolve::sotpMoveCallback(const std_msgs::Bool::ConstPtr& msg)
{
    stopMove();
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
                Adata.capturePoint = (Cstate.capturePoint + 1) % 4 + 1;
                Adata.space = 2;
                Cstate.canRotateFace1 = Adata.capturePoint + 2;
                Cstate.canRotateFace2 = 1;
                break;
            case 2:
                Adata.capturePoint = (Cstate.capturePoint + 1) % 4 + 1;
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
        isSwopOver = false;
        Adata.captureRobot = (Cstate.captureRobot + 1)%2;
        // Cstate.captureRobot = Adata.captureRobot;
        // Cstate.capturePoint = Adata.capturePoint;
    }
    Adata.angle = angle;
    Adata.otherRobot = (Adata.captureRobot + 1)%2;
}


void RubikCubeSolve::spinOnce()
{

    if(isBegingSolve)
    {
        if(runModel == 1 || runModel == 4)
        {
            ROS_INFO("start");
            photograph();
            setRobotEnable();
            ros::Duration(1.0).sleep();
        }
        if(runModel == 2 || runModel == 4)
        {
            ROS_INFO("get solve data");
            cubeParse::SolveCube srv;
            receiveSolve.call(srv);
        }
        if(runModel == 3 || runModel == 4)
        {
            ROS_INFO("begin solve .....");
            int cnt = 1;
            std_msgs::Int8MultiArray msg;
            msg.data.resize(2);
            msg.data[0] = rubikCubeSolvetransformData.size();
            for(int i=0; i<rubikCubeSolvetransformData.size() && ros::ok() && !isStop; ++i)
            {
                ROS_INFO("begin step: %d, count: %d",cnt, rubikCubeSolvetransformData.size());
                analyseData(rubikCubeSolvetransformData[i][0], rubikCubeSolvetransformData[i][1]);
                action();
                cnt ++;
                msg.data[1] = i+1;
                progressPub.publish(msg);
                if(cnt % 10 == 0)
                {
                    setRobotEnable();
                }
            }
            std::vector<std::vector<int> >().swap(rubikCubeSolvetransformData);
            Cstate.isFinish = true;
        }
        if(Cstate.isFinish || isPlaceCube)
        {
            if(Cstate.isFinish)
                placeCubeRobot = Cstate.captureRobot;
            placeCube();
        }
        Cstate.isFinish = false;
        isBegingSolve = false;
        runModel = 0;
    }
}
//TODO 抓取 拧魔方
void RubikCubeSolve::action()
{
    if(Adata.isSwop)
    {
        // 拧魔方
        swop(getMoveGroup(Adata.captureRobot), getMoveGroup(Adata.otherRobot), robotPose[Adata.captureRobot][Adata.capturePoint]);
    }
    // MOVE TO THE TARGET
    step(getMoveGroup(Adata.captureRobot), getMoveGroup(Adata.otherRobot), robotPose[Adata.captureRobot][Adata.capturePoint]);
}

//放置 魔方
bool RubikCubeSolve::swop(moveit::planning_interface::MoveGroupInterface& capture_move_group,\
                        moveit::planning_interface::MoveGroupInterface& rotate_move_group,\
                        geometry_msgs::PoseStamped pose)
{
    openGripper(capture_move_group);

    setAndMoveClient(capture_move_group, pose);       

    robotMoveCartesianUnitClient(capture_move_group, 0, pow(-1, Adata.captureRobot)*prepare_some_distance, 0);
    closeGripper(capture_move_group);

    openGripper(rotate_move_group);
    isSwopOver = true;
    Cstate.captureRobot = Adata.captureRobot;
    Cstate.capturePoint = Adata.capturePoint;

    robotMoveCartesianUnitClient(rotate_move_group, 0, pow(-1, Adata.captureRobot)*prepare_some_distance, 0);
    setAndMoveClient(rotate_move_group, robotPose[Adata.otherRobot][0]);
    return true;
}

// step 魔方提起
bool RubikCubeSolve::step(moveit::planning_interface::MoveGroupInterface& capture_move_group,\
                moveit::planning_interface::MoveGroupInterface& rotate_move_group,\
                geometry_msgs::PoseStamped pose)
{
    if(Adata.space == UP)
    {
        setAndMoveClient(capture_move_group, robotPose[Adata.captureRobot][3]);
        setAndMoveClient(capture_move_group, robotPose[Adata.captureRobot][6]);
        rotateCube(rotate_move_group, robotPose[Adata.otherRobot][7], Adata.angle);
    }
    else
    {
        rotateCube(rotate_move_group, robotPose[Adata.otherRobot][5], Adata.angle);
    }
    // 拧魔方的回原位
    if(Adata.space == UP)
    {
        robotMoveCartesianUnitClient(rotate_move_group, 0, -pow(-1, Adata.otherRobot)*prepare_some_distance*0.7071067, -0.7071067*prepare_some_distance);
    }
    else
    {
        robotMoveCartesianUnitClient(rotate_move_group, 0, -pow(-1, Adata.otherRobot)*prepare_some_distance, 0);
    }
    setAndMoveClient(rotate_move_group, robotPose[Adata.otherRobot][0]);
    // 抓住魔方的回原位
    pose.pose.position.y += pow(-1, Adata.captureRobot)*prepare_some_distance;
    if(Adata.space == UP)
        setAndMoveClient(capture_move_group, robotPose[Adata.captureRobot][3]);
    setAndMoveClient(capture_move_group, pose);
    return true;
}

//TODO rotateCube
bool RubikCubeSolve::rotateCube(moveit::planning_interface::MoveGroupInterface& rotate_move_group, 
                                geometry_msgs::PoseStamped pose, int angle)
{
    openGripper(rotate_move_group);
    setAndMoveClient(rotate_move_group, pose);
    if(Adata.space == UP)
    {
        robotMoveCartesianUnitClient(rotate_move_group, 0, pow(-1, Adata.otherRobot)*prepare_some_distance*0.7071067, 0.7071067*prepare_some_distance);
    }
    else
    {
        robotMoveCartesianUnitClient(rotate_move_group, 0, pow(-1, Adata.otherRobot)*prepare_some_distance*0.7071067, 0);
    }
    closeGripper(rotate_move_group);
    setJoint6ValueClient(rotate_move_group, angle);
    openGripper(rotate_move_group);
    return true;
}

bool RubikCubeSolve::openGripper(moveit::planning_interface::MoveGroupInterface& move_group)
{
    bool flag = true;
   hirop_msgs::openGripper srv;
   if(!isStop)
   {
        if(move_group.getName() == "arm0"){
            flag = openGripper_client0.call(srv);
            ROS_INFO("arm0 openGripper ");
        }
        else
        {
            ROS_INFO("arm1 openGripper ");
            flag = openGripper_client1.call(srv);
        }
        ros::Duration(1).sleep();
   }
    return flag;
}
bool RubikCubeSolve::closeGripper(moveit::planning_interface::MoveGroupInterface& move_group)
{
   bool flag = true;
   hirop_msgs::closeGripper srv;
   if(!isStop)
   {
        if(move_group.getName() == "arm0")
        {
            ROS_INFO("arm0 closeGripper ");
            flag = closeGripper_client0.call(srv);
        }
        else
        {
            ROS_INFO("arm1 closeGripper ");
            flag = closeGripper_client1.call(srv);
        }
        ros::Duration(1).sleep();
   }
    return flag;
}

void RubikCubeSolve::initPose()
{
    this->loadRobotPoses();
    // bool is_read_pose;
    // nh.getParam("/rubik_cube_solve/is_load_pose", is_read_pose);
    // if(is_read_pose)
    // {
    // }
    // else
    // {
    //     loadPoseData();
    //     this->getPoseStamped();
    //     writePoseFile();
    // }
    getPrepareSomeDistanceRobotPose();
}

void RubikCubeSolve::getPrepareSomeDistanceRobotPose()
{
    // sin45 = 0.7071067;
    const double cos45 = 0.7071067;
    for(int i=0; i<ROWS; ++i)
        for(int j=0; j<COLUMNS; ++j)
        {
            if(j==7)
            {
                robotPose[i][j].pose.position.z -= cos45*prepare_some_distance;
                robotPose[i][j].pose.position.y -= pow(-1, i)*prepare_some_distance*cos45;
                continue;
            }
            getPrepareSomeDistance(robotPose, i, j);
        }
}

inline void RubikCubeSolve::getPrepareSomeDistance(std::vector<std::vector<geometry_msgs::PoseStamped> >& pose, int row, int column)
{
    if(column != 0 && column != 6)
        pose[row][column].pose.position.y -= pow(-1, row)*prepare_some_distance;
}



void RubikCubeSolve::loadRobotPoses()
{
    std::vector<std::string> paramName={"/pose03_path", "/pose021_path", "/pose022_path", "/pose023_path",\
                                        "/pose024_path", "/pose025_path", "/pose011_path", "/pose012_path", \
                                        "/pose14_path", "/pose121_path", "/pose122_path", "/pose123_path", \
                                        "/pose124_path", "/pose125_path", "/pose111_path", "/pose112_path"};
    std::size_t cnt = 0;
    for (size_t i = 0; i < 2; i++)
    {
        for (size_t j = 0; j < 8; j++)
        {
            loadRobotPose(paramName[cnt], i, j);
            cnt++;
        }
    }
}

inline void RubikCubeSolve::loadRobotPose(std::string paramName, int row, int column)
{
    YAML::Node doc;
    std::string path;

    nh.getParam(paramName, path);
    doc = YAML::LoadFile(path);
    addData(robotPose[row][column], doc);
}

void RubikCubeSolve::loadPickPose()
{
    std::string path;
    YAML::Node doc;
    std::vector<std::string> pickPoseParam = {"/pick0_path", "/pick1_path", "/pick2_path",\
                                            "/pick3_path", "/pick4_path", "/pick5_path", \
                                            "/pick6_path", "/pick7_path", "pick8_path", "pick9_path"};
    photographPose.resize(pickPoseParam.size());
    for(int i=0; i < pickPoseParam.size(); ++i)
    {
        nh.getParam(pickPoseParam[i], path);
        doc = YAML::LoadFile(path);
        addData(photographPose[i], doc);
        if(i == 0 || i==9)
        {
            photographPose[i].pose.position.z += prepare_some_distance;
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

//TODO mofang
moveit::planning_interface::MoveItErrorCode RubikCubeSolve::setAndMove(moveit::planning_interface::MoveGroupInterface& move_group, \
                                                                        geometry_msgs::PoseStamped& poseStamped)
{
    // if(move_group.getName() == "arm1")
    //     ros::Duration(1.0).sleep();
    setJointAllConstraints(move_group);
    std::vector<double> joint = move_group.getCurrentJointValues();
    moveit_msgs::RobotState r;
    r.joint_state.position = joint;
    move_group.setStartState(r);
    move_group.setStartStateToCurrentState();

    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    move_group.setPoseTarget(poseStamped);
    moveit::planning_interface::MoveItErrorCode code;
    code = this->moveGroupPlanAndMove(move_group, my_plan);
    clearConstraints(move_group);
    return code;
}

moveit::planning_interface::MoveItErrorCode RubikCubeSolve::moveGroupPlanAndMove(moveit::planning_interface::MoveGroupInterface& move_group, \
                                                                moveit::planning_interface::MoveGroupInterface::Plan& my_plan)
{
    int cnt = 0;
    moveit::planning_interface::MoveItErrorCode code;
    if (!isStop)  
    do
    {
        code = move_group.plan(my_plan);
    }
    while (ros::ok() && cnt < 10 && code.val != moveit::planning_interface::MoveItErrorCode::SUCCESS && !isStop);
    if(code.val == moveit::planning_interface::MoveItErrorCode::SUCCESS && !isStop)
    {
        code = loop_move(move_group);
    }
    return code;
}

moveit::planning_interface::MoveItErrorCode RubikCubeSolve::loop_move(moveit::planning_interface::MoveGroupInterface& move_group)
{
    int cnt = 0;
    moveit::planning_interface::MoveItErrorCode code;
    do
    {
        ROS_INFO_STREAM("loop_move");
        code = move_group.move();
        cnt ++;
    }
    while (ros::ok() && cnt < 10 && code.val == moveit::planning_interface::MoveItErrorCode::TIMED_OUT && !isStop);
    return code;
}

geometry_msgs::PoseStamped RubikCubeSolve::setEulerAngle(moveit::planning_interface::MoveGroupInterface& move_group, double x, double y, double z, bool radian=true)
{
    return setEndEffectorPosoTarget(move_group, 0, 0, 0, x, y, z, radian, false);
}

// TODO  setEndEffectorPositionTarget
geometry_msgs::PoseStamped RubikCubeSolve::setEndEffectorPositionTarget(moveit::planning_interface::MoveGroupInterface& move_group, double x, double y, double z)
{
    geometry_msgs::PoseStamped pose;
    pose = setEndEffectorPosoTarget(move_group, x, y, z, 0, 0, 0, false, false);
    return pose;
}


//TODO setEndEffectorPosoTarget ROTATE
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
        setAndMoveClient(move_group, poseStamped);
    return poseStamped;
}

 inline double RubikCubeSolve::angle2rad(double& angle)
 {
    return (angle/180)*M_PI;
 }

// TODO 主要出现的bugb代码
void RubikCubeSolve::setJoint6Value(moveit::planning_interface::MoveGroupInterface& move_group, int angle,bool joint_mode)
{
    double a = static_cast<double>(angle);
    std::vector<double> joint;

    joint = move_group.getCurrentJointValues();
    moveit_msgs::RobotState r;
    r.joint_state.position = joint;
    move_group.setStartState(r);
    move_group.setStartStateToCurrentState();
    ROS_INFO_STREAM("joint_6 current:" << joint[5]);

    a = angle2rad(a);
    joint[5] += a;

    if((joint[5] > 5.0 || joint[5] < -5.0) && (angle == 180 || angle == -180))
    {
        ROS_INFO_STREAM("joint_6:" << joint[5]);
        joint[5] -= 2*a;
    }
    else if((joint[5] > 5.0 || joint[5] < -5.0) && (angle == 90 || angle == -90))
    {
        joint[5] -= 4*a;
    }

    ROS_INFO_STREAM("joint_6 reference:" << joint[5]);

    // joint 4
    // std::vector<std::string> data = move_group1.getJointNames();
    // int index = 0;
    // move_group.clearPoseTargets();
    // for(auto it :data){
    //     move_group.setJointValueTarget(it,joint[index]);
    //     index++;
    // }
    // 旧版
     move_group.setJointValueTarget(joint);

    /********************************************************/
    // 新版0.1
//    std::vector<geometry_msgs::Pose> waypoints;
//    geometry_msgs::PoseStamped temp_pose = move_group.getCurrentPose(move_group.getEndEffectorLink());
//    std::vector<double> data  = move_group.getCurrentRPY(move_group.getEndEffectorLink());

//    ROS_INFO_STREAM("R :" << data[2] +a);
//    geometry_msgs::Pose target_pose = temp_pose.pose;
//    tf::Quaternion q = tf::createQuaternionFromRPY(data[0],data[1], data[2] +a);

//    target_pose.orientation.w = q.w();
//    target_pose.orientation.x = q.x();
//    target_pose.orientation.y = q.y();
//    target_pose.orientation.z = q.z();

//    waypoints.push_back(target_pose);
//    moveit_msgs::RobotTrajectory trajectory;
//    const double jump_threshold = 0.0;
//    const double eef_step = 0.01;

//    while( move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory) < 0.8){
//        joint = move_group.getCurrentJointValues();
//        moveit_msgs::RobotState r;
//        r.joint_state.position = joint;
//        move_group.setStartState(r);
//        move_group.setStartStateToCurrentState();
//    }
//    moveit::planning_interface::MoveGroupInterface::Plan plan;
//	plan.trajectory_ = trajectory;
//    move_group.execute(plan);
    /*-------------------------------------*/

    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    // moveGroupPlanAndMove(move_group, my_plan);
     while (ros::ok() && !isStop)
     {
         if(move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS)
         {
             move_group.execute(my_plan);
             break;
         }
     }
}

int  RubikCubeSolve::writePoseFile()
{
    std::string path;
    std::vector<std::string> paramName={"/pose03_path", "/pose021_path", "/pose022_path", "/pose023_path",\
                                        "/pose024_path", "/pose025_path", "/pose011_path", "/pose012_path", \
                                        "/pose14_path", "/pose121_path", "/pose122_path", "/pose123_path", \
                                        "/pose124_path", "/pose125_path", "/pose111_path", "/pose112_path"};
    std::size_t cnt = 0;
    for (size_t i = 0; i < 2; i++)
    {
        for (size_t j = 0; j < 8; j++)
        {
            nh.getParam(paramName[cnt], path);
            writePoseOnceFile(path, robotPose[i][j]);
            cnt++;
        }
    }
}

bool RubikCubeSolve::writePoseOnceFile(const std::string& name, const geometry_msgs::PoseStamped& pose)
{
    std::ofstream fout(name, std::ios::out);
    YAML::Node config;
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


void RubikCubeSolve::robotMoveCartesianUnit2(moveit::planning_interface::MoveGroupInterface &group, double x, double y, double z)
{
    // if(group.getName() == "arm1")
    ros::Duration(1).sleep();
    std::vector<double>joint = group.getCurrentJointValues();
    moveit_msgs::RobotState r;
    r.joint_state.position = joint;
    group.setStartState(r);
    group.setStartStateToCurrentState();
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
    while( group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory) < 1.0 && !isStop && ros::ok());
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    plan.trajectory_ = trajectory;
    if(!isStop)
        group.execute(plan);
}

void RubikCubeSolve::backHome(int robot)
{
    if(!isStop)
    {
        ROS_INFO_STREAM("back home");
        const std::string home = "home" + std::to_string(robot);
        getMoveGroup(robot).setNamedTarget(home);
        getMoveGroup(robot).move();
    }
}

bool RubikCubeSolve::pickCube()
{
    setAndMoveClient(move_group1, photographPose[0]);
    openGripper(move_group1);
    robotMoveCartesianUnitClient(move_group1, 0, 0, -prepare_some_distance);
    closeGripper(move_group1);
    InitializationState();
    robotMoveCartesianUnitClient(move_group1, 0, 0, prepare_some_distance);
    return true;
}

int RubikCubeSolve::moveToPose()
{
    // system("rosrun rubik_cube_solve set_robot_enable_true.sh");
    // 复原魔方
    if(recordPointData.stepNum == 0)
    {
        overStepflag = 0;
        setAndMoveClient(move_group1, photographPose[0]);
        openGripper(move_group1);
        InitializationState();
    }
    // 1 + 17 = 18
    if(recordPointData.model == 0)
    {
        switch (recordPointData.stepNum)
        {
            case 1:
                // robot1
                closeGripper(move_group1);
                robotMoveCartesianUnitClient(move_group1, 0, 0, prepare_some_distance);
                setAndMoveClient(move_group1, robotPose[1][0]);
                break;
            case 2:
                // robot0
                setAndMoveClient(move_group0, robotPose[0][0]);
                break;
            case 3:
                // robot1
                setAndMoveClient(move_group1, robotPose[1][3]);
                robotMoveCartesianUnitClient(move_group1, 0, -prepare_some_distance, 0);
                break;
            case 4:
                setAndMoveClient(move_group0, robotPose[0][1]);
                break;
            case 5:
                robotMoveCartesianUnitClient(move_group0, 0, -prepare_some_distance, 0);
                setAndMoveClient(move_group0, robotPose[0][0]);
                setAndMoveClient(move_group0, robotPose[0][2]);
                break;
            case 6:
                robotMoveCartesianUnitClient(move_group0, 0, -prepare_some_distance, 0);
                setAndMoveClient(move_group0, robotPose[0][0]);
                setAndMoveClient(move_group0, robotPose[0][4]);
                break;
            case 7:
                robotMoveCartesianUnitClient(move_group0, 0, -prepare_some_distance, 0);
                setAndMoveClient(move_group0, robotPose[0][0]);
                setAndMoveClient(move_group0, robotPose[0][5]);
                break;
            case 8:
                robotMoveCartesianUnitClient(move_group0, 0, -prepare_some_distance, 0);
                setAndMoveClient(move_group0, robotPose[0][0]);
                setAndMoveClient(move_group1, robotPose[1][6]);
                break;
            case 9:
                setAndMoveClient(move_group0, robotPose[0][7]);
                break;
            case 10:
                robotMoveCartesianUnitClient(move_group0, 0, -prepare_some_distance*0.707, -prepare_some_distance*0.707);
                setAndMoveClient(move_group0, robotPose[0][0]);
                setAndMoveClient(move_group1, robotPose[1][3]);
                robotMoveCartesianUnitClient(move_group1, 0, -prepare_some_distance, 0);
                setAndMoveClient(move_group0, robotPose[0][3]);
                break;
            case 11:
                closeGripper(move_group0);
                openGripper(move_group1);
                robotMoveCartesianUnitClient(move_group1, 0, prepare_some_distance, 0);
                setAndMoveClient(move_group1, robotPose[1][0]);
                setAndMoveClient(move_group1, robotPose[1][1]);
                break;
            case 12:
                robotMoveCartesianUnitClient(move_group1, 0, prepare_some_distance, 0);
                setAndMoveClient(move_group1, robotPose[1][0]);
                setAndMoveClient(move_group1, robotPose[1][2]);
                break;
            case 13:
                robotMoveCartesianUnitClient(move_group1, 0, prepare_some_distance, 0);
                setAndMoveClient(move_group1, robotPose[1][0]);
                setAndMoveClient(move_group1, robotPose[1][4]);
                break;
            case 14:
                robotMoveCartesianUnitClient(move_group1, 0, prepare_some_distance, 0);
                setAndMoveClient(move_group1, robotPose[1][0]);
                setAndMoveClient(move_group1, robotPose[1][5]);
                break;
            case 15:
                robotMoveCartesianUnitClient(move_group1, 0, prepare_some_distance, 0);
                setAndMoveClient(move_group1, robotPose[1][0]);
                setAndMoveClient(move_group0, robotPose[0][6]);
                break;
            case 16:
                setAndMoveClient(move_group1, robotPose[1][7]);
                break;
            case 17:
                robotMoveCartesianUnitClient(move_group1, 0, prepare_some_distance*0.707, -prepare_some_distance*0.707);
                setAndMoveClient(move_group1, robotPose[1][0]);
                setAndMoveClient(move_group0, photographPose[9]);
                break;
        }
    }
    else if(recordPointData.model == 1)
    {
        switch (recordPointData.stepNum)
        {
            case 1:
                closeGripper(move_group1);
                robotMoveCartesianUnitClient(move_group1, 0, 0, prepare_some_distance);
                setAndMoveClient(move_group1, photographPose[1]);
                setAndMoveClient(move_group0, photographPose[2]);
                break;
            case 2:
                setAndMoveClient(move_group1, photographPose[3]);
                setAndMoveClient(move_group0, photographPose[4]);
                break;
            case 3:
                setAndMoveClient(move_group0, robotPose[0][0]);
                setAndMoveClient(move_group1, robotPose[1][3]);
                robotMoveCartesianUnitClient(move_group1, 0, -prepare_some_distance, 0);
                analyseData(3, 0);
                swop(getMoveGroup(Adata.captureRobot), getMoveGroup(Adata.otherRobot), robotPose[Adata.captureRobot][Adata.capturePoint]);
                setAndMoveClient(move_group0, photographPose[5]);
                setAndMoveClient(move_group1, photographPose[6]);
                break;
            case 4:
                setAndMoveClient(move_group0, photographPose[7]);
                setAndMoveClient(move_group1, photographPose[8]);
                break;
        }
    }    
    return 0;
}

int RubikCubeSolve::Cartesian()
{
    int f = 0x0001;
    f <<= recordPointData.stepNum;
    if(overStepflag >> recordPointData.stepNum)
        return 0;
    else
        overStepflag |= f;
    if(recordPointData.stepNum == 0)
        robotMoveCartesianUnitClient(move_group1, 0, 0, -prepare_some_distance);
    if(recordPointData.model == 0)
    {
        if((recordPointData.stepNum >= 4 && recordPointData.stepNum <=7) || recordPointData.stepNum == 10)
            robotMoveCartesianUnitClient(move_group0, 0, prepare_some_distance, 0);
        else if(recordPointData.stepNum >= 11 && recordPointData.stepNum <= 14)
            robotMoveCartesianUnitClient(move_group1, 0, -prepare_some_distance, 0);
        else if(recordPointData.stepNum == 9)
            robotMoveCartesianUnitClient(move_group0, 0, prepare_some_distance*0.707, prepare_some_distance*0.707);
        else if(recordPointData.stepNum == 16)
            robotMoveCartesianUnitClient(move_group1, 0, -prepare_some_distance*0.707, prepare_some_distance*0.707);
    }
    else if(recordPointData.stepNum == 17)
    {
        robotMoveCartesianUnitClient(move_group0, 0, 0, -prepare_some_distance);
    }
    return 0;
}

int RubikCubeSolve::updataPointData()
{
    // std::vector<int> storageLocation = {0, 0, 0, 3, 1, 2, 4, 5, 6, 7, 3, 1, 2, 4, 5, 6, 7, 9};
    if(recordPointData.stepNum == 0)
    {
        recordPose(1, recordPointData.poseName[recordPointData.stepNum], false);
    }
    else if(recordPointData.model == 0)
    {
        std::vector<int> robot0 = {2, 4, 5, 6, 7, 9, 10, 15, 17};
        std::vector<int> robot1 = {0, 1, 3, 8, 11, 12, 13, 14, 16};
        for(int i: robot0)
            if(recordPointData.stepNum == i)
            {
                recordPose(0, recordPointData.poseName[recordPointData.stepNum], false);
                return 0;
            }
        recordPose(1, recordPointData.poseName[recordPointData.stepNum], false);
        if(recordPointData.stepNum == 3)
        {
            geometry_msgs::PoseStamped newPoseStamped;
            newPoseStamped = move_group1.getCurrentPose();
            newPoseStamped.pose.position.y += prepare_some_distance;
            robotPose[1][3] = newPoseStamped;
        }
    }
    else if(recordPointData.model == 1)
    {
        int captureCubeRobot;
        int otherRobot;
        if(recordPointData.stepNum == 1 || recordPointData.stepNum == 2)
        {
            captureCubeRobot = 1;
            otherRobot = 0;
        }
        else if(recordPointData.stepNum == 3 || recordPointData.stepNum == 4)
        {
            captureCubeRobot = 0;
            otherRobot = 1;
        }
        recordPose(captureCubeRobot, recordPointData.shootPhotoPoseName[recordPointData.stepNum*2-2], false);
        recordPose(otherRobot, recordPointData.shootPhotoPoseName[recordPointData.stepNum*2-1], false);
    }
    return 0;
}
