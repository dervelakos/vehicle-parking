import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.actions import LifecycleNode

def generate_launch_description():
    # Get the package share directory
    pkg_share = get_package_share_directory('slam')

    # Define the launch configuration
    ld = LaunchDescription()

    # Define the parameters for the AMCL node
    amcl_params = os.path.join(pkg_share, 'config', 'amcl.yaml')

    # Define the map file
    map_file = os.path.join(pkg_share, 'maps', 'my_map.yaml')

    # Define the launch arguments
    ld.add_action(DeclareLaunchArgument('use_sim_time', default_value='false'))
    ld.add_action(DeclareLaunchArgument('map', default_value=map_file))

    # Start the AMCL node
    ld.add_action(LifecycleNode(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[amcl_params],
        remappings=[
            ('/odom', 'odom'),
            ('/scan', 'scan'),
            ('/map', 'map'),
        ],
        namespace=''
    ))

    # Start the map server node
    ld.add_action(Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{'yaml_filename': map_file}],
        remappings=[
            ('/map', 'map')
        ]
    ))

    # Start the lifecycle manager node
    ld.add_action(Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager',
        output='screen',
        parameters=[
            {'autostart': True},
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
            {'node_names': ['amcl', 'map_server']}
        ]
    ))

    return ld
