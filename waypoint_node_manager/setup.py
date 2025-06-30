from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'waypoint_node_manager'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kwon',
    maintainer_email='ksw7384@hanyang.ac.kr',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'waypoint_node_manager = waypoint_node_manager.waypoint_node_manager:main',
        ],
    },
)
