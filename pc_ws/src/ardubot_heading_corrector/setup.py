from glob import glob
import os
from setuptools import find_packages, setup

package_name = 'ardubot_heading_corrector'

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
    description='Correcteur de cap BNO055 entre Nav2 et le bridge ArduBot.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'heading_corrector = ardubot_heading_corrector.heading_corrector:main',
        ],
    },
)
