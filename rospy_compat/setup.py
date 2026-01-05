from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'rospy_compat'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Phyto-Arm Team',
    maintainer_email='developer@example.com',
    description='ROS1 rospy compatibility shim for ROS2',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [],
    },
)
