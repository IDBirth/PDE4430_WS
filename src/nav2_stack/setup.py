from setuptools import setup
import os
from glob import glob

package_name = 'nav2_stack'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
        data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch',
         glob('launch/*.py')),
        ('share/' + package_name + '/config',
         glob('config/*.yaml') + glob('config/*.rviz')),
        ('share/' + package_name + '/maps',
         glob('maps/*')),
    ],

    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Bilal',
    maintainer_email='bilalbaslar@gmail.com',
    description='Nav2 stack for my UGV',
    license='TODO: License',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [],
    },
)
