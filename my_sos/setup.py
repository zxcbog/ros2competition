from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'my_sos'

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
            'cv2_vis = my_sos.camera:main',
            'sos_vis = my_sos.sos:main',
            'velor = my_sos.velor:main'
        ],
    },
)
