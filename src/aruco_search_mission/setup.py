from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'aruco_search_mission'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Harinesh',
    maintainer_email='',
    description='Autonomous waypoint navigation and ArUco search',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mission_node = aruco_search_mission.mission_node:main',
        ],
    },
)