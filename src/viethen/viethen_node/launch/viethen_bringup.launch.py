#!/usr/bin/env python3
# Licensed under MIT. See LICENSE file. Copyright Carologistics.
import os
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import SetEnvironmentVariable
from launch.actions import GroupAction
from launch.actions import IncludeLaunchDescription
from launch.actions import OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def launch_nodes_withconfig(context, *args, **kwargs):

    # Get the launch directory
    bringup_dir = get_package_share_directory("viethen_node")
    launch_dir = os.path.join(bringup_dir, "launch")

    # Create the launch configuration variables
    namespace = LaunchConfiguration("namespace")
    use_sim_time = LaunchConfiguration("use_sim_time")
    launch_rviz = LaunchConfiguration("launch_rviz")
    launch_viethen = LaunchConfiguration("launch_viethen")
    launch_viethen_value = launch_viethen.perform(context).lower() in ["true", "1", "t", "y", "yes"]
    launch_simulation = LaunchConfiguration("launch_simulation")
    launch_simulation_value = launch_simulation.perform(context).lower() in ["true", "1", "t", "y", "yes"]
    launch_configuration = {}
    for argname, argval in context.launch_configurations.items():
        launch_configuration[argname] = argval

    # Specify the actions
    actions = [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, "moveit.launch.py")),
            launch_arguments={
                "namespace": namespace,
                "use_sim_time": use_sim_time,
                "launch_rviz": launch_rviz,
            }.items(),
        ),
    ]
    if launch_viethen_value:
        actions.extend(
            [
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(os.path.join(launch_dir, "viethen_node.launch.py")),
                    launch_arguments={
                        "namespace": namespace,
                        "use_sim_time": use_sim_time,
                        "launch_simulation": launch_simulation
                    }.items(),
                ),
            ]
        )
    if launch_simulation_value:
        actions.extend(
            [
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(os.path.join(launch_dir, "robot.launch.py")),
                    launch_arguments={
                        "namespace": namespace,
                        "use_sim_time": use_sim_time,
                    }.items(),
                )
            ]
        )
    bringup_cmd_group = GroupAction(actions)

    return [bringup_cmd_group]

def generate_launch_description():
    # Declare the launch arguments
    stdout_linebuf_envvar = SetEnvironmentVariable("RCUTILS_LOGGING_BUFFERED_STREAM", "1")

    declare_namespace_cmd = DeclareLaunchArgument("namespace", default_value="", description="Top-level namespace")

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Use simulation (Gazebo) clock if true",
    )

    declare_log_level_cmd = DeclareLaunchArgument("log_level", default_value="info", description="log level")

    declare_launch_simulation_cmd = DeclareLaunchArgument(
        "launch_simulation",
        default_value="true",
        description="whether to launch webots simulation or not",
    )

    declare_launch_rviz_cmd = DeclareLaunchArgument(
        "launch_rviz",
        default_value="true",
        description="whether to launch rviz or not",
    )

    declare_launch_viethen_cmd = DeclareLaunchArgument(
        "launch_viethen",
        default_value="true",
        description="whether to launch viethen_node (only for developement)",
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    # Set environment variables
    ld.add_action(stdout_linebuf_envvar)

    # Declare the launch options
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_log_level_cmd)
    ld.add_action(declare_launch_simulation_cmd)
    ld.add_action(declare_launch_rviz_cmd)
    ld.add_action(declare_launch_viethen_cmd)

    # Add the actions to launch all of the navigation nodes
    ld.add_action(OpaqueFunction(function=launch_nodes_withconfig))

    return ld
