import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'mission_traffic_light'

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
    description='신호등 인식 미션',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'traffic_light_node = mission_traffic_light.traffic_light_node:main',
        ],
    },
)
