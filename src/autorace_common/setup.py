import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'autorace_common'

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
    description='공통 유틸리티(기하, 경로, JSON 메시지, 파라미터)',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
        ],
    },
)
