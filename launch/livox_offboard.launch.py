from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import FrontendLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, TextSubstitution
import os

def generate_launch_description():
    namespace = LaunchConfiguration("ns")
    init_x, init_y, init_z = os.environ.get("INIT_X"), os.environ.get("INIT_Y"), os.environ.get("INIT_Z")
    init_roll, init_pitch, init_yaw = os.environ.get("INIT_ROLL"), os.environ.get("INIT_PITCH"), os.environ.get("INIT_YAW")
    veh = os.environ.get("VEH_NAME")
    return LaunchDescription([
        # Declare launch arguments
        DeclareLaunchArgument('hostname', default_value='nuc6'),
        DeclareLaunchArgument('tgt_system', default_value='1.1'),
        DeclareLaunchArgument('ns', default_value=EnvironmentVariable("VEH_NAME")),
        DeclareLaunchArgument('fcu_url', default_value='/dev/ttyACM0:921600'),
        DeclareLaunchArgument('respawn_mavros', default_value='false'),

        # Static transform publishers
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='odom_to_mavros',
            arguments=['0', '0', '0', '-1.57', '3.14', '0', 'odom', 'world_mavros']
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='odom_to_mocap',
            arguments=[init_x, init_y, init_z, init_yaw, init_pitch, init_roll, 'world_mocap', 'odom'] 
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='map_to_odom',
            arguments=['0', '0', '0', '0', '0', '0', 'odom', 'map']
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_link_to_ns',
            arguments=['0', '0', '0', '0', '0', '0', f"/{veh}/base_link", '/base_link'] 
        ),
        # Launch the repub_mocap node
        Node(
            package='ros2_px4_stack',
            executable='repub_livox',
            name='repub_livox_py',
            namespace=namespace,
            output='screen',
        ),
    ])
