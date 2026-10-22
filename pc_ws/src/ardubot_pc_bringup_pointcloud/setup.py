from setuptools import setup
from glob import glob
name = 'ardubot_pc_bringup_pointcloud'
setup(name=name, version='1.0.0', packages=[name],
 data_files=[('share/ament_index/resource_index/packages', ['resource/' + name]),
 ('share/' + name, ['package.xml']),
 ('share/' + name + '/launch', glob('launch/*.py')),
 ('share/' + name + '/config', glob('config/*.yaml')),
 ('share/' + name + '/rviz', glob('rviz/*.rviz'))],
 install_requires=['setuptools'], maintainer='jled12', maintainer_email='maintainer@example.invalid',
 description='Minimal ArduBot runtime bringup', license='GPL-3.0-only')
