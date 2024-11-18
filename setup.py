from setuptools import find_packages, setup

package_name = 'ros2-px4-stack'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/mocap_offboard.launch.py']),
        ('share/' + package_name + '/launch', ['launch/offboard_square_example.launch.py']),
        ('share/' + package_name + '/launch', ['launch/dynus_mavros.launch.py']),

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='juanrached',
    maintainer_email='juanrached@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'base_mavros_interface = ros2-px4-stack.base_mavros_interface:main',
            'drone_vars = ros2-px4-stack.drone_vars:main', 
            'make_experiment_trajectories = ros2-px4-stack.make_experiment_trajectories:main',
            'my_transform_broadcaster = ros2-px4-stack.my_transform_broadcaster:main',
            'offboard_node = ros2-px4-stack.offboard_node:main',
            'publish_trajectories = ros2-px4-stack.publish_trajectories:main',
            'repub_mocap = ros2-px4-stack.repub_mocap:main',
            'spoofed_mocap = ros2-px4-stack.spoofed_mocap:main',
            'track_square_node = ros2-px4-stack.track_square_node:main',
            'track_dynus_traj = ros2-px4-stack.track_dynus_traj:main',
            'fix_livox_pose = ros2-px4-stack.fix_livox_pose:main',
            'bag_to_csv = ros2-px4-stack.bag_to_csv:main',
        ],
    },
)
