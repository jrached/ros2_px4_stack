from setuptools import find_packages, setup
from glob import glob 

package_name = 'ros2_px4_stack'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(),
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
            'base_mavros_interface = ros2_px4_stack.src.base_mavros_interface:main',
            'repub_mocap = ros2_px4_stack.transforms.repub_mocap:main',
            'repub_odom = ros2_px4_stack.transforms.repub_odom:main',
            'track_dynus_traj = ros2_px4_stack.src.track_dynus_traj:main',
            'track_gen_traj = ros2_px4_stack.src.track_gen_traj:main',
            'trajgen_offboard_node = ros2_px4_stack.src.trajgen_offboard_node:main',
            'dynus_offboard_node = ros2_px4_stack.src.dynus_offboard_node:main',
            'repub_livox = ros2_px4_stack.transforms.repub_livox:main',
            'get_init_pose = ros2_px4_stack.transforms.get_init_pose:main',
            'mocap_to_livox_frame = ros2_px4_stack.transforms.mocap_to_livox_frame:main',
            'there_and_back = ros2_px4_stack.trajectories.there_and_back:main',
        ],
    },
)
