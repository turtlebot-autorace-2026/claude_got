import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'autorace_main'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*')),
        (os.path.join('share', package_name, 'maps'), glob('maps/*')),
        (os.path.join('share', package_name, 'routes'), glob('routes/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='miracle',
    maintainer_email='dev@example.com',
    description='미션 매니저(메인 상태머신), cmd_vel 먹스, 전체 런치',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'mission_manager = autorace_main.mission_manager:main',
            'cmd_mux = autorace_main.cmd_mux:main',
        ],
    },
)
