from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'autorace_core_SkufLords'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'materials'), glob(os.path.join('materials', '*.png'))),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.py'))),
        (os.path.join('share', package_name, 'util'), glob(os.path.join('util', '*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='evrey',
    maintainer_email='nikitaevreev1917@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'detect = autorace_core_SkufLords.detect:main',
            'road_analyze = autorace_core_SkufLords.road_analyze:main',
            'velor = autorace_core_SkufLords.velor:main',
            'cv2_vis = autorace_core_SkufLords.camera:main',
            'lidar_greedy = autorace_core_SkufLords.lidar_greedy:main',
        ],
    },
)
