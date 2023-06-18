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
  auto move_group_node = std::make_shared<JointTrajectoryPublisher>("moveit_debug_3", node_options);

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

  // Orientation constrained planning
  geometry_msgs::msg::PoseStamped target_pose2 = get_relative_pose(current_pose, -0.3, 0.3, 0.0);

  moveit_msgs::msg::OrientationConstraint orientation_constraint;
  orientation_constraint.header.frame_id = move_group .getPoseReferenceFrame();
  orientation_constraint.link_name = move_group.getEndEffectorLink();

  orientation_constraint.orientation = current_pose.pose.orientation;
  orientation_constraint.absolute_x_axis_tolerance = 0.4;
  orientation_constraint.absolute_y_axis_tolerance = 0.4;
  orientation_constraint.absolute_z_axis_tolerance = 0.4;
  orientation_constraint.weight = 1.0;

  moveit_msgs::msg::Constraints orientation_constraints;
  orientation_constraints.orientation_constraints.emplace_back(orientation_constraint);

  // Plan
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  move_group.setPathConstraints(orientation_constraints);
  move_group.setPoseTarget(target_pose2);
  move_group.setPlanningTime(10.0);
  
  rclcpp::Time initial_time = move_group_node->get_clock()->now();
  bool success = (move_group.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
  rclcpp::Duration time_to_plan = move_group_node->get_clock()->now() - initial_time;

  RCLCPP_INFO(LOGGER, "Planning to pose %s", success ? "" : "FAILED");
  RCLCPP_INFO_STREAM(LOGGER, "Planning took " << time_to_plan.nanoseconds()*1e-9 << "seconds");


  // Publish trajectory to be executed
  move_group_node->traj_publisher->publish(plan.trajectory_.joint_trajectory);

  // Shutdown and finish
  rclcpp::shutdown();
  return 0;
}
