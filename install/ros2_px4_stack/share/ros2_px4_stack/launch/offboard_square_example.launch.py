from launch import LaunchDescription  
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os 

def generate_launch_description():
    return LaunchDescription([
        # Declare launch arguments
        DeclareLaunchArgument('argname', default_value='val'),
        DeclareLaunchArgument('hostname', default_value='nuc1'),
        DeclareLaunchArgument('tgt_system', default_value='1.1'),
        DeclareLaunchArgument('ns', default_value='PX01'),
        DeclareLaunchArgument('fcu_url', default_value='/dev/ttyACM0:921600'),
        DeclareLaunchArgument('respawn_mavros', default_value='false'),
        Node(
            package='ros2_px4_stack',
            executable='track_square_node',
            name='track_square_node_py',
            output='screen',
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='mocap_to_mavros_tf',
            arguments=['0', '0', '0', '-1.57', '3.14', '0', 'world_mocap', 'world_mavros']
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='mocap_to_world',
            arguments=['0', '0', '0', '0', '0', '0', 'world', 'world_mocap']
        ),
        Node(
            package='ros2_px4_stack',
            executable='repub_mocap',
            name='repub_mocap_py',
            output='screen',
        ),
    ])