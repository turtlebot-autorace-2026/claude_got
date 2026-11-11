import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'mission_parking'

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
    description='주차 빈칸 판별 미션',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'parking_node = mission_parking.parking_node:main',
        ],
    },
)
