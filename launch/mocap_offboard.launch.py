from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.substitutions import LaunchConfiguration, PythonExpression, EnvironmentVariable
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import FrontendLaunchDescriptionSource
import os

def generate_launch_description():
    namespace = LaunchConfiguration("ns")
    return LaunchDescription([
        # Declare launch arguments
        DeclareLaunchArgument('hostname', default_value='nuc6'),
        DeclareLaunchArgument('tgt_system', default_value='1.1'),
        DeclareLaunchArgument('ns', default_value=EnvironmentVariable("VEH_NAME")),
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
            package='ros2_px4_stack',
            executable='repub_mocap',
            name='repub_mocap_py',
            namespace=namespace,
            output='screen',
            # You can include a condition to make it required if needed
            # condition=LaunchConfigurationEquals('some_condition', 'true')
        ),
    ])
