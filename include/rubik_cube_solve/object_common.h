#pragma once
#include <ros/ros.h>

#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>

// TF2
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/PoseStamped.h>
#include <moveit_msgs/CollisionObject.h>

double angle2rad(double& angle)
{
    return (angle/180)*3.1415;
}


void addCollisionObjects(ros::Publisher& planning_scene_diff_publisher, moveit_msgs::PlanningScene& p,
double sx, double sy, double sz,\
double px, double py, double pz, \
double ox, double oy, double oz, \
std::string frame_id, std::string id)
{

    std::vector<moveit_msgs::CollisionObject> collision_objects;
    collision_objects.resize(1);

    collision_objects[0].id = id;
    collision_objects[0].header.frame_id = frame_id;

    collision_objects[0].primitives.resize(1);
    collision_objects[0].primitives[0].type = collision_objects[0].primitives[0].BOX;
    collision_objects[0].primitives[0].dimensions.resize(3);
    collision_objects[0].primitives[0].dimensions[0] = sx;
    collision_objects[0].primitives[0].dimensions[1] = sy;
    collision_objects[0].primitives[0].dimensions[2] = sz;

    collision_objects[0].primitive_poses.resize(1);
    collision_objects[0].primitive_poses[0].position.x = px;
    collision_objects[0].primitive_poses[0].position.y = py;
    collision_objects[0].primitive_poses[0].position.z = pz;

    tf2::Quaternion orientation;
    orientation.setRPY(angle2rad(ox), angle2rad(oy), angle2rad(oz));
    collision_objects[0].primitive_poses[0].orientation = tf2::toMsg(orientation);

    collision_objects[0].operation = collision_objects[0].ADD;

    // planning_scene_interface.applyCollisionObjects(collision_objects);


    p.world.collision_objects.push_back(collision_objects[0]);
    p.is_diff = true;
    p.robot_state.is_diff = true;
    // p.object_colors.push_back();
    planning_scene_diff_publisher.publish(p);
    ros::Duration(1).sleep();
}

void addCollisionObjects(std::vector<moveit_msgs::CollisionObject> &collision_objects_group,
                                                std::vector<std::vector<double>>& dataGroup,
                                                std::string frame_id, std::string id)
{

  collision_objects_group.resize(dataGroup.size());
  int index = 0;

  for(auto it : dataGroup){


      collision_objects_group[index].id = id+std::to_string(index);
      collision_objects_group[index].header.frame_id = frame_id;

      double sx = it[0];double sy = it[1];double sz = it[2];
      double px = it[3];double py = it[4];double pz = it[5];
      double ox = it[6];double oy = it[7];double oz = it[8];
      collision_objects_group[index].primitives.resize(1);
      collision_objects_group[index].primitives[0].type = collision_objects_group[0].primitives[0].BOX;

      collision_objects_group[index].primitives[0].dimensions.resize(3);
      collision_objects_group[index].primitives[0].dimensions[0] = sx;
      collision_objects_group[index].primitives[0].dimensions[1] = sy;
      collision_objects_group[index].primitives[0].dimensions[2] = sz;

      collision_objects_group[index].primitive_poses.resize(1);
      collision_objects_group[index].primitive_poses[0].position.x = px;
      collision_objects_group[index].primitive_poses[0].position.y = py;
      collision_objects_group[index].primitive_poses[0].position.z = pz;

      tf2::Quaternion orientation;
      orientation.setRPY(angle2rad(ox), angle2rad(oy), angle2rad(oz));

      collision_objects_group[index].primitive_poses[0].orientation = tf2::toMsg(orientation);

      collision_objects_group[index].operation = collision_objects_group[index].ADD;

      index++;
  }
  ROS_INFO("object add ok...");
}


