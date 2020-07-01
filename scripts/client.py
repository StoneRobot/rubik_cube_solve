#! /usr/bin/env python
#coding=utf-8

import rospy
from rb_msgAndSrv.srv import rb_ArrayAndBool, rb_ArrayAndBoolRequest, rb_ArrayAndBoolResponse
from std_srvs.srv import Empty, EmptyRequest, EmptyResponse

rospy.init_node("cli_test")
client0 = rospy.ServiceProxy("magic_move_to_point", rb_ArrayAndBool)
client1 = rospy.ServiceProxy("magic_step_move", Empty)
client2 = rospy.ServiceProxy("magic_recordPose", Empty)
# a = rb_ArrayAndBool()
# a.data = 
# a = Empty()
# print("a", a)
# b = EmptyRequest()
# print("b", b)
for i in range(18):
    print("i: %s"%(i))
    print(client0.call([1, i]))
    rospy.sleep(2)
    client1.call()
    client2.call()
    rospy.sleep(3)
