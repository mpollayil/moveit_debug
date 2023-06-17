import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, OpaqueFunction, GroupAction, SetLaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node, PushRosNamespace
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument, OpaqueFunction, GroupAction, SetLaunchConfiguration
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.parameter_descriptions import ParameterFile

from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
   
    # MoveIt config
    moveit_config = (
        MoveItConfigsBuilder('rs013n')
        .robot_description(file_path=os.path.join('config', 'rs013n' + ".urdf.xacro"))
        .planning_scene_monitor(
            publish_robot_description=True, publish_robot_description_semantic=True
        )
        .planning_pipelines(
            pipelines=["ompl", "pilz_industrial_motion_planner"]
        )
        .to_moveit_configs()
    )

    # RViz config
    rviz_config = PathJoinSubstitution(
        [FindPackageShare('rs013n' + "_description"), "config", "demo.rviz"]
    )

    # MoveIt: start the actual move_group node/action server
    run_move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict()],
    )

    # Static TF from world to robot base link (This needed for MoveIt to show plans in RViz)
    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        output="log",
        arguments=["0.0", "0.0", "0.0", "0.0",
                   "0.0", "0.0", "world", '_base_link'],
    )

    # RViz
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        output="log",
        arguments=["-d", rviz_config],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
        ],
    )

    trajectory_publisher = Node(
        package='trajectory_publisher',
        executable='trajectory_publisher',
        output='screen',
        emulate_tty=True
    )

    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        output='screen',
        emulate_tty=True,
        parameters=[
            {'source_list': [f"/simulated_joint_states"]}
        ]
    )

    # Publish TF
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[moveit_config.robot_description],
    )

    return LaunchDescription([
        run_move_group_node,
        static_tf,
        rviz_node,
        trajectory_publisher,
        joint_state_publisher,
        robot_state_publisher,
    ])