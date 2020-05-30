#include <ros/ros.h>

#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>

// TF2
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/PoseStamped.h>
#include <moveit_msgs/CollisionObject.h>
#include <std_srvs/SetBool.h>
#include "rubik_cube_solve/object_common.h"

bool newData = false;
int flag = 1;

//ros::Publisher planning_scene_diff_publisher;
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
    flag = 1;
    return rep.success = true;
}

void applyObject(ros::Publisher& planning_scene_diff_publisher, moveit_msgs::PlanningScene &planning_scene){

  std::vector<std::vector<double>> data({{2.0, 2.0, 0.01, 0.0, 0.5, 1.00, 0.0, 0.0, 0.0},
                                         {0.01, 1.8, 1.5, 0.75, 0.4, 1.5, 0.0, 0.0, 0.0},
                                        //  {0.01, 2.0, 1.5, -0.38, 0.3, 1.5, 0.0, 0.0, 0.0},
                                         {2.0, 0.01, 1.5, 0.0, 1.5, 1.5, 0.0, 0.0, 0.0},
                                         {2.0, 0.01, 1.5, 0.0, -0.45, 1.5, 0.0, 0.0, 0.0},
                                         {0.1, 0.1, 0.105, 0.5, 0.67, 1.0675, 0.0, 0.0, 0.0}});
  ROS_INFO("applyObject....");
  std::vector<moveit_msgs::CollisionObject> obj;
  addCollisionObjects(obj, data, "world","box");
  for(auto it :obj)
    planning_scene.world.collision_objects.push_back(it);
  planning_scene.is_diff = true;
  planning_scene.robot_state.is_diff = true;
  planning_scene_diff_publisher.publish(planning_scene);
}


void removeAllobject(ros::Publisher& planning_scene_diff_publisher, moveit_msgs::PlanningScene &planning_scene){

   for(int i = 0; i<  6; i++)
   {
       moveit_msgs::CollisionObject remove_object;
       remove_object.id = "box"+std::to_string(i);
       remove_object.header.frame_id = "world";
       remove_object.operation = remove_object.REMOVE;
       planning_scene.world.collision_objects.push_back(remove_object);
   }
   planning_scene_diff_publisher.publish(planning_scene);
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "apply_object");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(1);
    spinner.start();
    ros::ServiceServer ser = nh.advertiseService("remove_object", callBack);
//    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    moveit_msgs::PlanningScene planning_scene;

    ros::Publisher planning_scene_diff_publisher = nh.advertise<moveit_msgs::PlanningScene>("planning_scene", 1);
    ros::WallDuration sleep_t(0.5);
    while (planning_scene_diff_publisher.getNumSubscribers() < 1)
     {
        sleep_t.sleep();
    }
    ROS_INFO("waiting ok.");
    // removeAllobject(planning_scene_diff_publisher,planning_scene);
    while (ros::ok())
    {
        if(flag == 1){
            if(newData)
                removeAllobject(planning_scene_diff_publisher,planning_scene);
            else
                applyObject(planning_scene_diff_publisher,planning_scene);

            flag = -flag;
        }
        ros::Duration(0.5).sleep();
    }
    
    return 0;
}
