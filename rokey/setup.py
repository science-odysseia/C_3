import os 
from glob import glob
from setuptools import find_packages, setup

package_name = 'rokey'

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
    maintainer='rokey',
    maintainer_email='rokey@todo.todo',
    description='TODO: Package description',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'simple_move = rokey.move:main',
            'initiallize = rokey.initiallize:main',
            'gear = rokey.gear:main',
            'grip_test = rokey.grip_test:main',
            'modelhouse = rokey.lego_modelhouse:main',
            'subscriber = rokey.centers_subscriber:main',
            'blocks = rokey.9points_blocks:main',
            'studs = rokey.9points_blocks_studs:main',
            'corner = rokey.9points_blocks_corner:main',
            'test = rokey.test:main',
        ],
    },
)
