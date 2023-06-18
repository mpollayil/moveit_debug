#include <cmath>
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

// Creates a pose at a given positional offset from the current pose
geometry_msgs::msg::PoseStamped get_relative_pose(geometry_msgs::msg::PoseStamped current_pose,
                                                  double x, double y, double z)
{

  auto target_pose = current_pose;
  target_pose.pose.position.x += x;
  target_pose.pose.position.y += y;
  target_pose.pose.position.z += z;

  return target_pose;
};

// Creates a pose at a given positional offset from the current pose
std::pair<double, double> is_close(geometry_msgs::msg::Pose pose_1, geometry_msgs::msg::Pose pose_2)
{

  double pos_norm = (pow(pose_1.position.x - pose_2.position.x, 2) 
    + pow(pose_1.position.y - pose_2.position.y, 2) + pow(pose_1.position.z - pose_2.position.z, 2));
  double rot_norm = (pow(pose_1.orientation.x - pose_2.orientation.x, 2) 
    + pow(pose_1.orientation.y - pose_2.orientation.y, 2) + pow(pose_1.orientation.z - pose_2.orientation.z, 2) 
    + pow(pose_1.orientation.w - pose_2.orientation.w, 2));

  return {pos_norm, rot_norm};
};