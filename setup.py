from setuptools import find_packages, setup

package_name = 'uwb_drone_experiments'

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
            'base_mavros_interface = uwb_drone_experiments.base_mavros_interface:main',
            'drone_vars = uwb_drone_experiments.drone_vars:main', 
            'make_experiment_trajectories = uwb_drone_experiments.make_experiment_trajectories:main',
            'my_transform_broadcaster = uwb_drone_experiments.my_transform_broadcaster:main',
            'offboard_node = uwb_drone_experiments.offboard_node:main',
            'publish_trajectories = uwb_drone_experiments.publish_trajectories:main',
            'repub_mocap = uwb_drone_experiments.repub_mocap:main',
            'spoofed_mocap = uwb_drone_experiments.spoofed_mocap:main',
            'track_square_node = uwb_drone_experiments.track_square_node:main',
            'track_dynus_traj = uwb_drone_experiments.track_dynus_traj:main',
            'fix_livox_pose = uwb_drone_experiments.fix_livox_pose:main',
            'bag_to_csv = uwb_drone_experiments.bag_to_csv:main',
        ],
    },
)
