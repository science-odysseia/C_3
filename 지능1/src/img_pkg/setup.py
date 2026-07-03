from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'img_pkg'

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
            'img_raw1= img_pkg.image_viewer:main',
            'img_comp= img_pkg.image_comp:main',
            'img_yolo= img_pkg.image_yolo:main',
            'img_depth= img_pkg.image_depth:main',
            'img_record= img_pkg.image_record:main',
            'img_photo= img_pkg.image_photo:main',
            'img_depth_comp= img_pkg.image_depth_comp:main',
        ],
    },
)
