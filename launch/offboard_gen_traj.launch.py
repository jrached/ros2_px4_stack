from launch import LaunchDescription  
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration, EnvironmentVariable
import os 

def generate_launch_description():
    namespace = LaunchConfiguration("ns")
    return LaunchDescription([
        # Declare launch arguments
        DeclareLaunchArgument('argname', default_value='val'),
        DeclareLaunchArgument('hostname', default_value='nuc6'),
        DeclareLaunchArgument('tgt_system', default_value='1.1'),
        DeclareLaunchArgument('ns', default_value=EnvironmentVariable("VEH_NAME")),
        DeclareLaunchArgument('fcu_url', default_value='/dev/ttyACM0:921600'),
        DeclareLaunchArgument('respawn_mavros', default_value='false'),
        DeclareLaunchArgument('odom_type', default_value='mocap'),

        Node(
            package='ros2_px4_stack',
            executable='track_gen_traj',
            name='track_gen_traj_py',
            namespace=namespace,
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
            executable='repub_odom',
            name='repub_odom_py',
            namespace=namespace,
            output='screen',
            parameters=[{
                '~odom_type': LaunchConfiguration('odom_type'),
            }]
        ),
    ])