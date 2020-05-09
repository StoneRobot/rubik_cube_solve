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

void addCollisionObjects(moveit::planning_interface::PlanningSceneInterface& planning_scene_interface, 
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

  planning_scene_interface.applyCollisionObjects(collision_objects);
}
int main(int argc, char *argv[])
{
    ros::init(argc, argv, "add");
    ros::NodeHandle nh;
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    addCollisionObjects(planning_scene_interface, 
    atof(argv[1]), atof(argv[2]), atof(argv[3]), 
    atof(argv[4]), atof(argv[5]), atof(argv[6]), 
    atof(argv[7]), atof(argv[8]), atof(argv[9]), 
    argv[10], argv[11]);
    return 0;
}