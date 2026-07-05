from setuptools import find_packages, setup

package_name = 'turtlebot4_beep'

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
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'beep_node = turtlebot4_beep.beep_node:main',
            'lidar_subscriber = turtlebot4_beep.lidar_subscriber:main',
            'image_subscriber = turtlebot4_beep.image_subscriber:main',
            'image_publisher = turtlebot4_beep.image_pub:main',
            'data_subscriber = turtlebot4_beep.data_sub:main',
            'data_publisher = turtlebot4_beep.data_pub:main',
            'beep_test = turtlebot4_beep.beep_test:main',
            'beep_finale = turtlebot4_beep.beep_finale:main',
        ],
    },
)
