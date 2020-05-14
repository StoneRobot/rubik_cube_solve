#include <ros/ros.h>

#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>

// TF2
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/PoseStamped.h>
#include <moveit_msgs/CollisionObject.h>
#include <std_srvs/SetBool.h>

bool before = false;
bool newData = false;

void removeOrAddObject(moveit::planning_interface::PlanningSceneInterface& planning_scene_interface, bool isRemove)
{
    std::vector<moveit_msgs::CollisionObject> collision_objects;
    collision_objects.resize(1);
    if(isRemove)
    {   
        collision_objects[0].id = "wall";
        collision_objects[0].operation = collision_objects[0].REMOVE;
        planning_scene_interface.applyCollisionObjects(collision_objects);
    }
    else
    {

        collision_objects[0].id = "wall";
        collision_objects[0].header.frame_id = "base_link";

        collision_objects[0].primitives.resize(1);
        collision_objects[0].primitives[0].type = collision_objects[0].primitives[0].BOX;
        collision_objects[0].primitives[0].dimensions.resize(3);
        //"2 2 0.01 0 0.5 0.05
        collision_objects[0].primitives[0].dimensions[0] = 2;
        collision_objects[0].primitives[0].dimensions[1] = 2;
        collision_objects[0].primitives[0].dimensions[2] = 0.01;

        collision_objects[0].primitive_poses.resize(1);
        collision_objects[0].primitive_poses[0].position.x = 0;
        collision_objects[0].primitive_poses[0].position.y = 0.5;
        collision_objects[0].primitive_poses[0].position.z = 0;

        tf2::Quaternion orientation;
        orientation.setRPY(0, 0, 0);
        collision_objects[0].primitive_poses[0].orientation = tf2::toMsg(orientation);

        collision_objects[0].operation = collision_objects[0].ADD;

        planning_scene_interface.applyCollisionObjects(collision_objects);
    }
    
}

bool callBack(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& rep)
{

    newData = req.data;
    return rep.success = true;
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "remove_object");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(1);
    spinner.start();
    ros::ServiceServer ser = nh.advertiseService("remove_objec", callBack);
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    while (ros::ok())
    {
        if(newData != before)
        {
            removeOrAddObject(planning_scene_interface, newData);
            before = newData;
        }
        ros::Duration(0.5).sleep();
    }
    
    return 0;
}
