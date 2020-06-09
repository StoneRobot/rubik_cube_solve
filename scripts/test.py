#! /usr/bin/env python
#coding=utf-8

import rospy
from rubik_cube_solve.srv import rubik_cube_solve_cmd
import random
from os import system

rospy.init_node("rubik_cube_slove_test")
client = rospy.ServiceProxy("/analyse_rubik_cube_cmd", rubik_cube_solve_cmd)
rospy.wait_for_service("analyse_rubik_cube_cmd")
rospy.Duration(5)
rate = rospy.Rate(1)
random.seed()
cnt = 0
k = 0
client.call(0, 0)
while not rospy.is_shutdown():
    i = random.randint(1, 6)
    j = random.choice([90, -90, 180])
    if k == i:
        continue
    rospy.loginfo("face: %d; angle: %d", i, j)
    client.call(i, j)
    cnt += 1
    if cnt % 10 == 0:
        system("rosrun rubik_cube_solve set_robot_enable_false.sh")
        system("rosrun rubik_cube_solve set_robot_enable_true.sh")
    rospy.loginfo("cnt: %d", cnt)
    rate.sleep()
    k = i
