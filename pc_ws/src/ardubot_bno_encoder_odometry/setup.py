from glob import glob
import os
from setuptools import find_packages, setup

package_name = 'ardubot_bno_encoder_odometry'

setup(
    name=package_name,
    version='1.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Bastien',
    maintainer_email='maintainer@example.invalid',
    description='Odometrie hybride pour ArduBot: distance moyenne encodeurs et cap BNO055.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'bno_encoder_odometry = ardubot_bno_encoder_odometry.bno_encoder_odometry:main',
        ],
    },
)
