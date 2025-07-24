from setuptools import find_packages, setup
from glob import glob
package_name = 'webserver_interface_ros2'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/json_schemas', glob('json_schemas/*')),
        ('share/' + package_name + '/config', glob('config/*')),
        ('share/' + package_name + '/nodes', glob('nodes/*')),
        ('share/' + package_name + '/maps', glob('maps/*')),
        ('share/' + package_name + '/launch', glob('launch/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='asdasd0111@naver.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'webserver_interface = webserver_interface_ros2.webserver_interface:main',
            'ros_handler = webserver_interface_ros2.ros_handler:main',
            'node_manager = webserver_interface_ros2.node_manager:main',
            'test = webserver_interface_ros2.test:main',
            'service_test = webserver_interface_ros2.service_test:main',
            'instantaction_test = webserver_interface_ros2.instantaction_test:main',
        ],
    },
)
