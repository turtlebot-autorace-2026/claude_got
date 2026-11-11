import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'lane_slam'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), [f for f in glob('config/*') if os.path.isfile(f)]),
        (os.path.join('share', package_name, 'models'), [f for f in glob('models/*') if os.path.isfile(f)]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='miracle',
    maintainer_email='dev@example.com',
    description='카메라 차선 기반 SLAM(맵핑/파티클필터 위치추정)',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'lane_slam_node = lane_slam.lane_slam_node:main',
            'lane_map_viewer = lane_slam.map_viewer:main',
        ],
    },
)
