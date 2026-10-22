import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'car_control'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='wahabsafdar42@gmail.com',
    description='Lidar-based obstacle avoider and safety gate for the car.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'obstacle_avoider = car_control.obstacle_avoider:main',
            'safety_gate = car_control.safety_gate:main',
        ],
    },
)
