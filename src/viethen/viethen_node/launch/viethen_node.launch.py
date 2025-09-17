import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Get the directory of your package
    package_dir = get_package_share_directory('viethen_node')

    # Path to the YAML configuration file
    config_file_path = os.path.join(package_dir, 'config', 'viethen_node.yaml')

    # Declare launch arguments with default values from the YAML file
    declare_launch_simulation = DeclareLaunchArgument(
        'launch_simulation', default_value='false', description='Launch with simulation'
    )

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='false', description='Use CAD file for robot configuration'
    )

    declare_namepsace = DeclareLaunchArgument(
        'namespace', default_value='/', description='Path to CAD file'
    )

    # Launch configurations for overriding
    launch_simulation = LaunchConfiguration('launch_simulation')
    use_sim_time = LaunchConfiguration('use_sim_time')
    namespace = LaunchConfiguration('namespace')

    # Node to be launched
    viethen_node = Node(
        package='viethen_node',
        executable='viethen_node',
        name='viethen_node',
        output='screen',
        namespace=namespace,
        parameters=[
            config_file_path,  # Load parameters from the YAML file
            {
                'launch_simulation': launch_simulation,
                'use_sim_time': use_sim_time,
            }
        ],
    )

    return LaunchDescription([
        declare_launch_simulation,
        declare_use_sim_time,
        declare_namepsace,
        viethen_node,
    ])
