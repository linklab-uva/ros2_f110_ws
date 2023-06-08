from setuptools import setup
from glob import glob
import os

package_name = 'pure_pursuit'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'lines'), glob('lines/*.csv')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='deepracer',
    maintainer_email='deepracer@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'adaptive_lookahead = pure_pursuit.adaptive_lookahead:main',
            'find_nearest_goal = pure_pursuit.find_nearest_goal:main',
            'find_nearest_pose = pure_pursuit.find_nearest_pose:main',
            'vehicle_controller = pure_pursuit.vehicle_controller:main',
            'waypoint_logger = pure_pursuit.waypoint_logger:main'
        ],
    },
)
