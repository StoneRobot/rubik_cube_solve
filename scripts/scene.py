#! /usr/bin/env python
#coding=utf-8

import rospy
from sensor_msgs.msg import JointState
from os import system

intoCallback = 0
def callBack(data):
    global intoCallback
    if intoCallback == 0:
        print(data.name[3], data.position[3])
        print(data.name[9], data.position[9])
    if data.position[3] < 0  or data.position[9] > 0:
        system("rosnode kill /rubik_cube_solve")
        intoCallback = 1
        # system("rosnode kill /monitor")


rospy.init_node("monitor", anonymous=True)
sub = rospy.Subscriber("joint_states", JointState,callBack)
rospy.loginfo("start")
rospy.spin()
