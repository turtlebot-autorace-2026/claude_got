import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'autorace_sim_tools'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='miracle',
    maintainer_email='dev@example.com',
    description='시뮬레이션 전용 도구(맵핑용 가상 텔레옵, 평가, 하우스로봇 배치)',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'mapping_driver = autorace_sim_tools.mapping_driver:main',
            'race_monitor = autorace_sim_tools.race_monitor:main',
            'spawn_house_robot = autorace_sim_tools.spawn_house_robot:main',
        ],
    },
)
