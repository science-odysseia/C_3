from setuptools import find_packages, setup

package_name = 'day3'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
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
            'nav_to_pose = day3.nav_to_pose:main',
            'nav_through_poses = day3.nav_through_poses:main',
            'follow_waypoints = day3.follow_waypoints:main',
            'create_path = day3.create_path:main',
            'depth_checker = day3.depth_checker:main',
            'depth_to_3d_ts = day3.depth_to_3d_ts:main',
            'depth_to_3d = day3.depth_to_3d:main',
            'depth_to_nav_goal = day3.depth_to_nav_goal:main',
            'depth_comp_checker = day3.depth_comp_checker:main',
        ],
    },
)
