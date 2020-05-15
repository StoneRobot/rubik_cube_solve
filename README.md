# rubik_cube_solve
# 启动机器人
左边的为0号机器人, 右边为1号机器人
roslaunch hsr_bringup co605_dual_arm_real.launch 启动真实机器人
roslaunch co605_dual_arm_gripper_moveit_config demo.launch 虚拟

# 复原魔方节点
roslaunch rubik_cube_solve solve.launch
# 进行标定使用的服务, move_group为机器人编号, 记录文件在recordPose,里面有点位名字说明,记录JointSpace的功能不可用
    rosservice call /record_pose  
    
# true或false都是启动,只是给一个信号
    rostopic pub -1 /beginSolve std_msgs/Bool "data: false" 

# 测试链接
ping 10.10.56.214 1号机器人
ping 192.168.98.2 0号机器人

# 障碍物
roslaunch rubik_cube_solve add.launch 添加障碍物
rosrun rubik_cube_solve remove_object 

# gripper
roslaunch gripper_bridge gripper_bridge_dual.launch
rosrun gripper_bridge gripper.sh

# 使能与复位
rosrun rubik_cube_solve clear_robot_fault.sh  复位
rosrun rubik_cube_solve set_robot_enable_false.sh 下使能
rosrun rubik_cube_solve set_robot_enable_true.sh 上

# 测试
rosrun rubik_cube_solve test.py 测试脚本

# note
注意gripper的线

git status
git pull 
git add -A
git commit -m '描述'
git push original master
git checkout 
git branch -a
git log
