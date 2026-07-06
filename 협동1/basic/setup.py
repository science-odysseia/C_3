from setuptools import find_packages, setup

package_name = 'basic'

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
            'basic_upper = basic.basic_upper:main',
            'joystick = basic.joystick:main',
            'joystick_keyboard = basic.joystick_keyboard:main',
            'movej_test = basic.movej_test:main',
            'force_control = basic.force_control:main',
            'rotation_self = basic.rotation_self:main',
            'rotation_async = basic.rotation_async:main',
            'rotation_canceling = basic.rotation_canceling:main',
            'mix_force_rotate = basic.mix_force_rotate:main',
            'button = basic.button:main',
        ],
    },
)
