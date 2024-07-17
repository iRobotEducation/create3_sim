from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'axis_camera'

setup(
    name=package_name,
    version='2.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Ekumen',
    maintainer_email='ekumen@irbt.onmicrosoft.com',
    description='Provides common launch and configuration scripts for a simulated iRobot(R) Create(R) 3 Educational Robot.',
    license='BSD',
    tests_require=['pytest'],
    entry_points={
    },
)
