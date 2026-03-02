from setuptools import find_packages, setup

package_name = 'marine_charts_to_gazebo_world'

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
    ],
    maintainer='Roland Arsenault',
    maintainer_email='roland@ccom.unh.edu',
    description='Generate Gazebo Harmonic SDF worlds from S57 ENC chart data',
    license='BSD-2-Clause',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'generate_world = marine_charts_to_gazebo_world.generate_world:main',
        ],
    },
)
