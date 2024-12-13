from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import FrontendLaunchDescriptionSource
import os

def generate_launch_description():
    return LaunchDescription([
        # Declare launch arguments
        DeclareLaunchArgument('hostname', default_value='nuc1'),
        DeclareLaunchArgument('tgt_system', default_value='1.1'),
        DeclareLaunchArgument('ns', default_value='PX01'),
        DeclareLaunchArgument('fcu_url', default_value='/dev/ttyACM0:921600'),
        DeclareLaunchArgument('respawn_mavros', default_value='false'),

        # Static transform publishers
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='mocap_to_mavros_tf',
            arguments=['0', '0', '0', '-1.57', '3.14', '0', 'odom', 'world_mavros']
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='mocap_to_world',
            arguments=['0', '0', '0', '0', '0', '0', 'world', 'world_mocap']
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='map_to_world',
            arguments=['0', '0', '0', '0', '0', '0', 'world', 'map']
        ),

        # Launch the repub_mocap node
        Node(
            package='ros2_px4_stack',
            executable='repub_livox',
            name='repub_livox_py',
            output='screen',
            # You can include a condition to make it required if needed
            # condition=LaunchConfigurationEquals('some_condition', 'true')
        ),
    ])
