from setuptools import find_packages, setup

package_name = 'finalproject'

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
            'mode = finalproject.mode:main',
            'battery_percentage = finalproject.battery_percentage:main',
            'patrol = finalproject.patrol:main',
            'yolo_detection = finalproject.yolo_detection:main',
            'dock_test = finalproject.dock_test:main',
            'time_sub = finalproject.time_sub:main',
            'time_pub = finalproject.time_pub:main',
            'mode_time_ver = finalproject.mode_time_ver:main',
            'UI_bat_limit = finalproject.UI_bat_limit:main',
            'report_targeting = finalproject.report_targeting:main',
            'beep_infinite = finalproject.beep_infinite:main',
            'warning_pub = finalproject.warning_pub:main',
            'to_sector = finalproject.to_sector:main',
            'gohome = finalproject.gohome:main',
            'undock = finalproject.undock:main',
            'robot3_underbattery = finalproject.robot3_underbattery:main',
        ],
    },
)
