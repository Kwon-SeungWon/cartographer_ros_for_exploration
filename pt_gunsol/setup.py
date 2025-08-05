from setuptools import find_packages, setup

package_name = 'pt_gunsol'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'rclpy', 'std_msgs', 'sensor_msgs', 'nav_msgs'], # 종속성 추가. 실제 메시지 타입에 맞게 수정
    zip_safe=True,
    maintainer='tch',
    maintainer_email='think.code.help@gmail.com',
    description='ROS2 subscriber node for multiple topics.',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            'subscriber_service_all = pt_gunsol.subscriber_service_all:main',
        ],
    },
)
