import os

from glob import glob

from setuptools import find_packages, setup

package_name = 'portsmouth_nh_gazebo'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    install_requires=['setuptools'],
    zip_safe=True,
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
    ],
    maintainer='Roland Arsenault',
    maintainer_email='roland@ccom.unh.edu',
    description='Gazebo Harmonic world for Portsmouth NH harbor area',
    license='BSD-2-Clause',
    tests_require=['pytest'],
)
