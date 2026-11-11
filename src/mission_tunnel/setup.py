import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'mission_tunnel'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), [f for f in glob('config/*') if os.path.isfile(f)]),
        (os.path.join('share', package_name, 'launch'), [f for f in glob('launch/*') if os.path.isfile(f)]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='miracle',
    maintainer_email='dev@example.com',
    description='터널: LiDAR SLAM 맵 + Nav2 주행',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'tunnel_nav_node = mission_tunnel.tunnel_nav_node:main',
            'tunnel_map_align = mission_tunnel.map_align:main',
        ],
    },
)
