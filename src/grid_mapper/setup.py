import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'grid_mapper'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='xutianq999',
    maintainer_email='xutianq999@example.com',
    description='Growing 2D occupancy grid from Super-LIO registered scans',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'grid_mapper = grid_mapper.grid_mapper_node:main',
        ],
    },
)
