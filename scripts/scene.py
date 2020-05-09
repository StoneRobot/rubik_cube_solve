#! /usr/bin/env python
#coding=utf-8

import rospy
import tf
import tf2_geometry_msgs
from moveit_commander import PlanningSceneInterface
from moveit_msgs.msg import PlanningScene
from geometry_msgs.msg import PoseStamped

def add_collision_object(s, sx, sy, sz, px, py, pz, ox, oy, oz, id):
    q = PoseStamped()
    q.pose.orientation = tf.transformations.quaternion_from_euler(ox, oy, oz)
    q.header.frame_id = "world"
    q.pose.position.x = px
    q.pose.position.y = py
    q.pose.position.z = pz
    # q.pose.orientation.w = 1
    size = [sx, sy, sz]
    s.add_box(id, q, size)

rospy.init_node("add_collision")
scene = PlanningSceneInterface()
scene_pub = rospy.Publisher("planning_scene", PlanningScene, queue_size=5)
add_collision_object(scene, 0.01, 2, 2, 0, 0, 1, 0, 0, 0, "1")
add_collision_object(scene, 0.01, 2, 2, 0, 0, 1, 0, 0, 0, "2")
add_collision_object(scene, 0.01, 2, 2, 0, 0, 1, 0, 0, 0, "3") 