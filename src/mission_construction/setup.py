import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'mission_construction'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='miracle',
    maintainer_email='dev@example.com',
    description='LiDAR 실시간 장애물 회피(공사구간)',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'avoidance_node = mission_construction.avoidance_node:main',
        ],
    },
)
