from setuptools import find_packages, setup

package_name = 'pt_gunsol_instant_actions'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=[
        'setuptools',  # Python 패키지 관리 및 빌드 도구
        'fastapi',  # Python 기반의 웹 프레임워크 (API 개발)
        'uvicorn',  # ASGI 서버 (FastAPI 애플리케이션 실행)
        'pyzmq',  # ZeroMQ 메시징 라이브러리의 Python 바인딩
        'loguru',  # Log 관리
    ], # Python에 해당하는 내용만, ROS는 package.xml에서 정의
    zip_safe=True,
    maintainer='tch',
    maintainer_email='think.code.help@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            'publisher_service_all = pt_gunsol_instant_actions.publisher_service:main',
        ],
    },
)
