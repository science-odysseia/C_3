from setuptools import find_packages, setup
import os
from glob import glob


package_name = 'miniproject'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='yswbulb',
    maintainer_email='ysw414153@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'img_yolo = miniproject.image_yolo:main',
            'img_yolo_higher = miniproject.image_yolo_higher:main',
            'nav_to_pose = miniproject.nav_to_pose:main',
            'follow_waypoints = miniproject.follow_waypoints:main',
            'follow_waypoints_stop = miniproject.follow_waypoints_stop:main',
            'infinite_tracking = miniproject.infinite_tracking:main',
            'temp_navgoal = miniproject.temp_navgoal:main',
            'depth_track_navgoal = miniproject.depth_track_navgoal:main',
            'white_percentage = miniproject.white_percentage:main',
            'back_home = miniproject.back_home:main',
            'finale = miniproject.finale:main',
        ],
    },
)
