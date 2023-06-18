#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>

#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <moveit_msgs/msg/collision_object.hpp>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("move_group_demo");

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

class JointTrajectoryPublisher : public rclcpp::Node
{
  public:
    JointTrajectoryPublisher(const std::string &node_name, rclcpp::NodeOptions options) : Node(node_name, options)
    {
      traj_publisher = this->create_publisher<trajectory_msgs::msg::JointTrajectory>("joint_trajectory", 10);
      ps_publisher = this->create_publisher<moveit_msgs::msg::PlanningScene>("planning_scene", 10);
    }

    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr traj_publisher;
    rclcpp::Publisher<moveit_msgs::msg::PlanningScene>::SharedPtr ps_publisher;

  private:
    // Nothing private
};

int main(int argc, char **argv)
{
  // Initialize ROS node and spin
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  auto move_group_node = std::make_shared<JointTrajectoryPublisher>("moveit_debug", node_options);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(move_group_node);
  std::thread([&executor]()
              { executor.spin(); })
      .detach();

  // MoveGroupInteface and PlanningSceneInterface
  static const std::string PLANNING_GROUP = "rs013n_arm";

  moveit::planning_interface::MoveGroupInterface move_group(move_group_node, PLANNING_GROUP);
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  // Define goal pose
  geometry_msgs::msg::Pose target_pose1;
  target_pose1.orientation.w = 1.0;
  target_pose1.position.x = 0.5;
  target_pose1.position.y = 0.5;
  target_pose1.position.z = 1.0;
  move_group.setPoseTarget(target_pose1);

  // Add obstacle to the environment
  moveit_msgs::msg::PlanningScene planning_scene;  

  moveit_msgs::msg::CollisionObject collision_object;
  collision_object.header.frame_id = move_group.getPlanningFrame();

  collision_object.id = "box";

  shape_msgs::msg::SolidPrimitive primitive;
  primitive.type = primitive.BOX;
  primitive.dimensions.resize(3);
  primitive.dimensions[primitive.BOX_X] = 0.1;
  primitive.dimensions[primitive.BOX_Y] = 1.5;
  primitive.dimensions[primitive.BOX_Z] = 0.5;

  geometry_msgs::msg::Pose box_pose;
  box_pose.orientation.w = 1.0;
  box_pose.position.x = 0.48;
  box_pose.position.y = 0.0;
  box_pose.position.z = 0.25;

  collision_object.primitives.push_back(primitive);
  collision_object.primitive_poses.push_back(box_pose);
  collision_object.operation = collision_object.ADD;

  std::vector<moveit_msgs::msg::CollisionObject> collision_objects;
  collision_objects.push_back(collision_object);

  planning_scene.world.collision_objects = collision_objects;
  // planning_scene_.robot_state.attached_collision_objects = attached;

  // move_group_node->ps_publisher->publish(planning_scene);
  planning_scene_interface.addCollisionObjects(collision_objects);
  sleep(1.0); // Wait for being sure planning scene published

  // Plan now
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  bool success = (move_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);

  RCLCPP_INFO(LOGGER, "Planning to pose %s", success ? "" : "FAILED");

  // Publish trajectory to be executed
  move_group_node->traj_publisher->publish(my_plan.trajectory_.joint_trajectory);
  
  // Shutdown and finish
  rclcpp::shutdown();
  return 0;
}
