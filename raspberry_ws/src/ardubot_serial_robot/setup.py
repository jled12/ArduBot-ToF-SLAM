from setuptools import setup

package_name = 'ardubot_serial_robot'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/serial_robot.launch.py']),
        ('share/' + package_name + '/config', ['config/serial_robot.yaml']),
    ],
    install_requires=['setuptools', 'pyserial'],
    zip_safe=True,
    maintainer='jled12',
    maintainer_email='maintainer@example.invalid',
    description='Serial bridge for ArduBot: cmd_vel, odom, tf and BNO055 IMU.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'serial_robot = ardubot_serial_robot.serial_robot:main',
        ],
    },
)
