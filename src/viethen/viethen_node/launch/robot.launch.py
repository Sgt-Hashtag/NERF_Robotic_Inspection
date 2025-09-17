#!/usr/bin/env python

import os
import launch
import pathlib
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions.path_join_substitution import PathJoinSubstitution
from webots_ros2_driver.webots_launcher import WebotsLauncher
from webots_ros2_driver.webots_controller import WebotsController
from webots_ros2_driver.wait_for_controller_connection import WaitForControllerConnection
from webots_ros2_driver.urdf_spawner import URDFSpawner, get_webots_driver_node
from webots_ros2_driver.utils import controller_url_prefix




def generate_launch_description():
    simulation_dir = get_package_share_directory("viethen_simulation")
    viethen_description_dir = get_package_share_directory("viethen_description")

    # Starts Webots
    webots = WebotsLauncher(
        world=PathJoinSubstitution([simulation_dir, 'worlds', 'viethen_mfg_simulation.wbt']),
        mode="realtime",
        ros2_supervisor=True
    )

    ur5e_description_path = os.path.join(viethen_description_dir, 'urdf', 'ur5e_with_camera.urdf')
    #robot_description = pathlib.Path(ur5e_urdf_path).read_text()
    ros2_control_params = os.path.join(viethen_description_dir, 'config', 'ros2_controllers.yaml')
    
    #Driver nodes
    universal_robot_driver = WebotsController(
        robot_name='ur5e',
        parameters=[
            {'robot_description': ur5e_description_path},
            {'use_sim_time': True},
            {'set_robot_state_publisher': True},
            ros2_control_params
        ],
    )

    # Other ROS 2 nodes
    controller_manager_timeout = ['--controller-manager-timeout', '100']
    controller_manager_prefix = 'python.exe' if os.name == 'nt' else ''
    trajectory_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        output='screen',
        prefix=controller_manager_prefix,
        arguments=['ur5e_planning_grp_controller'] + controller_manager_timeout,
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        output='screen',
        prefix=controller_manager_prefix,
        arguments=['ur5e_joint_state_broadcaster'] + controller_manager_timeout,
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': '<robot name=""><link name=""/></robot>'}],
    )

    ros_control_spawners = [trajectory_controller_spawner, joint_state_broadcaster_spawner]

    # Wait for the simulation to be ready to start RViz, the navigation and spawner
    waiting_nodes = WaitForControllerConnection(
        target_driver= universal_robot_driver ,
        nodes_to_start=ros_control_spawners
    )
    
    return LaunchDescription([
        webots,
        webots._supervisor,

        # This action will kill all nodes once the Webots simulation has exited
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=webots,
                on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
            )
        ),
        
        # Other ROS 2 nodes
        robot_state_publisher,
        universal_robot_driver,
        waiting_nodes, 

        # Kill all the nodes when the driver node is shut down
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=universal_robot_driver,
                on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
            )
        ),
    ])
