#!/usr/bin/env python3

import os
import pathlib
import yaml
from launch.actions import LogInfo
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory, get_packages_with_prefixes
from launch.actions import IncludeLaunchDescription, LogInfo
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition

def generate_launch_description():
    launch_description_nodes = []
    viethen_description = get_package_share_directory("viethen_description")
    robot_description_path = pathlib.Path(os.path.join(viethen_description, "urdf", "ur5e_with_camera.urdf")).read_text()
    robot_semantic_path = pathlib.Path(os.path.join(viethen_description, "urdf", "ur5e_with_camera.srdf")).read_text()

    def load_file(filename):
        return pathlib.Path(os.path.join(viethen_description, 'config', filename)).read_text()

    def load_yaml(filename):
        return yaml.safe_load(load_file(filename))
    
    # Check if moveit is installed
    if 'moveit' in get_packages_with_prefixes():
        # Configuration
        description = {'robot_description': robot_description_path}
        description_semantic = {'robot_description_semantic': robot_semantic_path}
        description_kinematics = {'robot_description_kinematics': load_yaml('kinematics.yaml')}
        sim_time = {'use_sim_time': True}

        # Launch arguments
        declare_launch_rviz_cmd = DeclareLaunchArgument(
            "launch_rviz",
            default_value="true",
            description="whether to launch rviz or not",
        )
        launch_description_nodes.append(declare_launch_rviz_cmd)

        # Rviz node
        rviz_config_file = os.path.join(viethen_description, 'config', 'moveit.rviz')

        launch_description_nodes.append(
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                arguments=['-d', rviz_config_file],
                condition=IfCondition(LaunchConfiguration('launch_rviz')),
                parameters=[
                    description,
                    description_semantic,
                    description_kinematics,
                    sim_time
                ],
            )
        )

        # MoveIt2 node
        movegroup = {'move_group': load_yaml('moveit_movegroup.yaml')}
        moveit_controllers = {
            'moveit_controller_manager': 'moveit_simple_controller_manager/MoveItSimpleControllerManager',
            'moveit_simple_controller_manager': load_yaml('moveit_controllers.yaml')
        }

        launch_description_nodes.append(
            Node(
                package='moveit_ros_move_group',
                executable='move_group',
                output='screen',
                parameters=[
                    description,
                    description_semantic,
                    description_kinematics,
                    moveit_controllers,
                    movegroup,
                    sim_time
                ],
            )
        )

        # Camera node (optional)
        # launch_description_nodes.append(
        #     Node(
        #         package='viethen_node',
        #         executable='webots_camera_node',
        #         name='webots_camera_node',
        #         output='screen',
        #         condition=IfCondition(LaunchConfiguration('launch_camera')),
        #         parameters=[sim_time],
        #     )
        # )

        # Scan coordinator (optional)
        # launch_description_nodes.append(
        #     Node(
        #         package='viethen_node',
        #         executable='scan_coordinator',
        #         name='scan_coordinator',
        #         output='screen',
        #         condition=IfCondition(LaunchConfiguration('launch_coordinator')),
        #         parameters=[sim_time],
        #     )
        # )

    else:
        launch_description_nodes.append(LogInfo(msg='"moveit" package is not installed, \
                                                please install it in order to run this demo.'))

    return LaunchDescription(launch_description_nodes)
