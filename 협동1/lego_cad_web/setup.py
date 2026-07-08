import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'lego_cad_web'

# static/ 아래 모든 파일을 재귀적으로 찾아서 share/<pkg>/static/... 로 설치
static_files = []
static_root = os.path.join(package_name, 'static')
for dirpath, _dirnames, filenames in os.walk(static_root):
    if not filenames:
        continue
    rel_dir = os.path.relpath(dirpath, package_name)  # e.g. "static" or "static/sub"
    install_dir = os.path.join('share', package_name, rel_dir)
    files = [os.path.join(dirpath, f) for f in filenames]
    static_files.append((install_dir, files))

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ] + static_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@example.com',
    description='PyQt LEGO CAD 빌더의 웹(Three.js) 버전을 ros2 run 으로 띄우는 패키지',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'lego_cad_web = lego_cad_web.web_server_node:main',
        ],
    },
)
