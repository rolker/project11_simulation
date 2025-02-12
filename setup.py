from glob import glob
import os
from setuptools import find_packages, setup

package_name = 'mbes_sim'

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
        ('share/' + package_name + '/data', glob(os.path.join('data', '*.tiff'))),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.yaml')))

    ],
    maintainer='Roland Arsenault',
    maintainer_email='roland@ccom.unh.edu',
    description='ROS2 Multibeam Echosounder simple simulator',
    license='BSD',
    entry_points={
        'console_scripts': [
            'mbes_sim = mbes_sim.mbes_sim:main',
        ],
    },
)
