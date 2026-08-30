from setuptools import setup, find_packages
from glob import glob

import os


package_name = 'cameranew_pkg'


setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name]
        ),
        (
            'share/' + package_name,
            ['package.xml']
        ),
        (
            os.path.join('share', package_name, 'launch'),
            glob(os.path.join('launch', '*launch.[pxy][yma]*'))
        ),
        (
            os.path.join('share', package_name, 'config'),
            glob(os.path.join('config', '*.yaml'))
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='yildizrover',
    maintainer_email='yildizrover@todo.todo',
    description='ZED kameralari, webcamler, IP kamera ve kamera yonetimi paketi',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'webcam = cameranew_pkg.webcam:main',
            'webcam2 = cameranew_pkg.webcam2:main',
            'dual_webcam = cameranew_pkg.dual_webcam:main',
            'zed_compressed_throttle = cameranew_pkg.zed_compressed_throttle:main',
            'camera_manager = cameranew_pkg.camera_manager:main',
            'ip_camera_publisher = cameranew_pkg.ip_camera_publisher:main',
            'science = cameranew_pkg.science:main',
            'science_web = cameranew_pkg.science_web:main',
            'gps_science = cameranew_pkg.gps_science:main',
            'can_scienceveri = cameranew_pkg.can_scienceveri:main',
            
        ],
    },
)
