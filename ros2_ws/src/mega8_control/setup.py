import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'mega8_control'

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
    description='ROS2 control package for Mega8 Arduino system with relay and sensor control via UDP',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'm8relay = mega8_control.m8relay:main',
            'm8sensor = mega8_control.m8sensor:main',
            'mega8_gui = mega8_control.mega8_gui:main',
        ],
    },
)
