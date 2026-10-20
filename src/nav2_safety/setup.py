import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'nav2_safety'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'docs'), glob('docs/*.md')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='xutianq999',
    maintainer_email='xutianq999@example.com',
    description='Safety layer: cmd_vel arbitration, collision monitor, localization watchdog',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'deadman = nav2_safety.deadman:main',
            'localization_watchdog = nav2_safety.localization_watchdog:main',
            'estop_latch = nav2_safety.estop_latch:main',
            'recovery_guard = nav2_safety.recovery_guard:main',
            'scan_qos_relay = nav2_safety.scan_qos_relay:main',
            'robot_status_monitor = nav2_safety.robot_status_monitor:main',
        ],
    },
)
