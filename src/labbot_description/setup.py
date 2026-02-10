from setuptools import setup
import os
from glob import glob

package_name = 'labbot_description'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*')),
        (os.path.join('share', package_name, 'meshes'), glob('meshes/*')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*'))
    ],
    install_requires=[
        'setuptools', 
        'xacro',               # Add xacro for processing Xacro files
        'robot_state_publisher',  # Add robot_state_publisher to load the robot model
        'gazebo-ros-pkgs'      # Add gazebo_ros for Gazebo integration with ROS 2
    ],
    zip_safe=True,
    maintainer='abhinav',
    maintainer_email='abhinav@todo.com',
    description='The ' + package_name + ' package',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
