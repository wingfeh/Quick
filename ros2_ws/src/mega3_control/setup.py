from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'mega3_control'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*')))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='wingfeh',
    maintainer_email='wingfehhui1@gmail.com',
    description='Control package for Arduino Mega 3',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'm3relay = mega3_control.m3relay:main',
            'm3sensor = mega3_control.m3sensor:main',
            'mega3_gui = mega3_control.mega3_gui:main',
        ],
    },
)
