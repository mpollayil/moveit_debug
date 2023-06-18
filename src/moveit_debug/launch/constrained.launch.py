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

    moveit_constrained = Node(
        package='moveit_debug',
        executable='moveit_constrained',
        output='screen',
        emulate_tty=True
    )

    return LaunchDescription([
        moveit_constrained
    ])