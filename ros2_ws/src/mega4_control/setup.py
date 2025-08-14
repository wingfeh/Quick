import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'mega4_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='wingfeh',
    maintainer_email='wingfeh@todo.todo',
    description='ROS2 package for controlling Arduino Mega4 relay via UDP',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'm4relay = mega4_control.m4relay:main',
            'mega4_gui = mega4_control.mega4_gui:main',
            'm4sensor = mega4_control.m4sensor:main',
        ],
    },
)
