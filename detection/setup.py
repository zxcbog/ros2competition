from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'detection'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'materials'), glob(os.path.join('materials', '*.png'))),
        (os.path.join('share', package_name, 'util'), glob(os.path.join('util', '*')))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='maxim',
    maintainer_email='1@d.ru',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'detect = detection.detect:main',
        ],
    },
)
