import os
import glob
from setuptools import setup

package_name = 'post_station'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        # Install all YAML instruction sets
        (
            f'share/{package_name}/config/instruction_sets',
            glob.glob(os.path.join('config', 'instruction_sets', '*.yaml'))
        ),
        # Install all launch files
        (
          f'share/{package_name}/launch',
          glob.glob(os.path.join('launch', '*.py'))
        ),

        # Required ROS packaging metadata
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='you@example.com',
    description='Station node for POST system',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'station = post_station.station:main',
            'parcel = post_station.parcel:main',
        ],
    },
)

