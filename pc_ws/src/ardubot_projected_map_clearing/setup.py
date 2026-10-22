from setuptools import setup
from glob import glob

package_name = 'ardubot_projected_map_clearing'

setup(
    name=package_name,
    version='0.3.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ArduBot',
    maintainer_email='maintainer@example.invalid',
    description='Persistent 2D occupancy grid using OctoMap projection and pose-based camera FOV',
    license='MIT',
    entry_points={
        'console_scripts': [
            'map_clearing_node = ardubot_projected_map_clearing.map_clearing_node:main',
        ]
    },
)
