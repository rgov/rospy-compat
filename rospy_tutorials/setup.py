from setuptools import setup
import os
from glob import glob

package_name = 'rospy_tutorials'

setup(
    name=package_name,
    version='2.0.0,
    packages=[],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Test Team',
    maintainer_email='rzg@example.com',
    description='ROS2 versions of rospy tutorials using rospy compatibility layer',
    license='BSD',
    tests_require=[],
)
