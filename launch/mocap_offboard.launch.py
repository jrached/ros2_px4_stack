from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import FrontendLaunchDescriptionSource
from launch.substitutions import GetPackageShareDirectory
import os

def generate_launch_description():
    return LaunchDescription([
        # Declare launch arguments
        DeclareLaunchArgument('hostname', default_value='nuc1'),
        DeclareLaunchArgument('tgt_system', default_value='1.1'),
        DeclareLaunchArgument('ns', default_value='PX01'),
        DeclareLaunchArgument('fcu_url', default_value='/dev/ttyACM0:921600'),
        DeclareLaunchArgument('respawn_mavros', default_value='false'),

        # Include the mavros launch file
        # IncludeLaunchDescription(
        #     [os.path.join(get_package_share_directory('mavros'), 'launch', 'px4.launch')],
        #     launch_arguments={
        #         'fcu_url': LaunchConfiguration('fcu_url'),
        #         'tgt_system': LaunchConfiguration('tgt_system'),
        #         'respawn_mavros': LaunchConfiguration('respawn_mavros'),
        #     }.items()
        # ),
        IncludeLaunchDescription(
            FrontendLaunchDescriptionSource(
                os.path.join(
                    GetPackageShareDirectory('mavros'), 
                    'launch', 
                    'px4.launch'
                )
            )
        ),

        # Static transform publishers
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
            package='tf2_ros',
            executable='static_transform_publisher',
            name='map_to_world',
            arguments=['0', '0', '0', '0', '0', '0', 'world', 'map']
        ),

        # Launch the repub_mocap node
        Node(
            package='uwb_drone_experiments',
            executable='repub_mocap',
            name='repub_mocap_py',
            output='screen',
            # You can include a condition to make it required if needed
            # condition=LaunchConfigurationEquals('some_condition', 'true')
        ),

        # Subscribe to broadcasted trajectories
        Node(
            package='uwb_drone_experiments',
            executable='offboard_node',
            name='offboard_node_py',
            output='screen',
            # Again, you can include a condition to make it required
        ),
    ])
