import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'route_follower'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), [f for f in glob('config/*') if os.path.isfile(f)]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='miracle',
    maintainer_email='dev@example.com',
    description='기록 경로 추종(Pure Pursuit)과 경로 기록기',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'route_follower_node = route_follower.follower_node:main',
            'route_recorder = route_follower.recorder_node:main',
            'route_tool = route_follower.route_tool:main',
            'route_mark = route_follower.route_mark:main',
        ],
    },
)
