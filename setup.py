from setuptools import find_packages, setup
from glob import glob 

package_name = 'ros2_px4_stack'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
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
            'base_mavros_interface = ros2_px4_stack.base_mavros_interface:main',
            'drone_vars = ros2_px4_stack.drone_vars:main', 
            'my_transform_broadcaster = ros2_px4_stack.my_transform_broadcaster:main',
            'offboard_node = ros2_px4_stack.offboard_node:main',
            'repub_mocap = ros2_px4_stack.repub_mocap:main',
            'track_square_node = ros2_px4_stack.track_square_node:main',
            'track_dynus_traj = ros2_px4_stack.track_dynus_traj:main',
            'track_gen_traj = ros2_px4_stack.track_gen_traj:main',
            'setpoint_offboard_node = ros2_px4_stack.setpoint_offboard_node:main',
            'trajgen_offboard_node = ros2_px4_stack.trajgen_offboard_node:main',
            'dynus_offboard_node = ros2_px4_stack.dynus_offboard_node:main',
            'repub_livox = ros2_px4_stack.repub_livox:main'
        ],
    },
)
