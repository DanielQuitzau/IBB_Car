from setuptools import setup
import os
from glob import glob

package_name = 'IBB_Car'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name, f'{package_name}.utils'],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'pyserial'],
    zip_safe=True,
    maintainer='rpiibb',
    maintainer_email='daniel.quitzau@haw-hamburg.de',
    description='IBB Car ROS2 Communication Node',
    license='MIT',
    entry_points={
        'console_scripts': [
            'serial_bridge = IBB_Car.serial_bridge:main',
        ],
    },
)
