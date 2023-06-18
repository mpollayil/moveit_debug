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

#include "utils/auxiliary_functions.hpp"

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
  auto move_group_node = std::make_shared<JointTrajectoryPublisher>("moveit_debug_2", node_options);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(move_group_node);
  std::thread([&executor]()
              { executor.spin(); })
      .detach();

  // MoveGroupInteface and PlanningSceneInterface
  static const std::string PLANNING_GROUP = "rs013n_arm";

  moveit::planning_interface::MoveGroupInterface move_group(move_group_node, PLANNING_GROUP);

  // Get current pose
  auto current_pose = move_group.getCurrentPose();

  // Constrained planning
  geometry_msgs::msg::PoseStamped target_pose2 = get_relative_pose(current_pose, -0.3, 0.3, 0.0);

  moveit_msgs::msg::PositionConstraint box_constraint;
  box_constraint.header.frame_id = move_group.getPoseReferenceFrame();
  box_constraint.link_name = move_group.getEndEffectorLink();
  shape_msgs::msg::SolidPrimitive box;
  box.type = shape_msgs::msg::SolidPrimitive::BOX;
  box.dimensions = {0.8, 0.8, 0.1};
  box_constraint.constraint_region.primitives.emplace_back(box);

  geometry_msgs::msg::Pose box_constraint_pose;
  box_constraint_pose.position.x = current_pose.pose.position.x;
  box_constraint_pose.position.y = current_pose.pose.position.y;
  box_constraint_pose.position.z = current_pose.pose.position.z;
  box_constraint_pose.orientation.w = 1.0;
  box_constraint.constraint_region.primitive_poses.emplace_back(box_constraint_pose);
  box_constraint.weight = 1.0;

  moveit_msgs::msg::Constraints box_constraints;
  box_constraints.position_constraints.emplace_back(box_constraint);

  // Plan
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  move_group.setPathConstraints(box_constraints);
  move_group.setPoseTarget(target_pose2);
  move_group.setPlanningTime(10.0);
  
  rclcpp::Time initial_time = move_group_node->get_clock()->now();
  bool success = (move_group.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
  rclcpp::Duration time_to_plan = move_group_node->get_clock()->now() - initial_time;

  RCLCPP_INFO(LOGGER, "Planning to pose %s", success ? "" : "FAILED");
  RCLCPP_INFO_STREAM(LOGGER, "Planning took " << time_to_plan.nanoseconds()*1e9 << "seconds");


  // Publish trajectory to be executed
  move_group_node->traj_publisher->publish(plan.trajectory_.joint_trajectory);

  // Shutdown and finish
  rclcpp::shutdown();
  return 0;
}
