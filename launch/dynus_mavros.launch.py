from launch import LaunchDescription  
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration, EnvironmentVariable 
import os 

def generate_launch_description():
    namespace = LaunchConfiguration("ns")
    init_x, init_y, init_z = os.environ.get("INIT_X"), os.environ.get("INIT_Y"), os.environ.get("INIT_Z")
    init_roll, init_pitch, init_yaw = os.environ.get("INIT_ROLL"), os.environ.get("INIT_PITCH"), os.environ.get("INIT_YAW")
    veh = os.environ.get("VEH_NAME")

    return LaunchDescription([
        # Declare launch arguments
        DeclareLaunchArgument('argname', default_value='val'),
        DeclareLaunchArgument('hostname', default_value='nuc6'),
        DeclareLaunchArgument('tgt_system', default_value='1.1'),
        DeclareLaunchArgument('ns', default_value=EnvironmentVariable("VEH_NAME")),
        DeclareLaunchArgument('fcu_url', default_value='/dev/ttyACM0:921600'),
        DeclareLaunchArgument('respawn_mavros', default_value='false'),
        
        # Run dynus node
        Node(
            package='ros2_px4_stack',
            executable='track_dynus_traj',
            name='track_dynus_traj_py',
            namespace=namespace,
            output='screen',
        ),

        # Static transforms
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='odom_to_mocap',
            arguments=[init_x, init_y, init_z, init_yaw, init_pitch, init_roll, 'world_mocap', f'{veh}/init_pose'] 
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='map_to_odom',
            arguments=['0', '0', '0', '0', '0', '0', 'world', 'world_mocap']
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='map_to_odom',
            arguments=['0', '0', '0', '0', '0', '0', 'world', 'map']
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_d455',
            arguments=['0', '0', '0', '0', '0', '0', f"{veh}/base_link", f"{veh}/d455_link"]
        ),

        # Run repub_livox node
        Node(
            package='ros2_px4_stack',
            executable='repub_livox',
            name='repub_livox_py',
            namespace=namespace,
            output='screen',
        ),

        # Run mocap to livox command frame conversion
        Node(
            package='ros2_px4_stack', 
            executable='mocap_to_livox_frame',
            name='mocap_to_livox_frame',
            namespace=namespace,
            output='screen', 
        )
    ])