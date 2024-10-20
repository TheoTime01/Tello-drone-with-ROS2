from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'drone_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='theotime',
    maintainer_email='theotime.perrichet@cpe.fr',
    description='Package to control the tello drone with a controller',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        	'control = drone_control.control:main',
        	'qr_code_follower = drone_control.qr_code_follower:main',
        	'tello_behavior = drone_control.tello_behavior:main',
        	'control_safe = drone_control.control_plus_secure:main',
        	'travelling = drone_control.travelling:main',
        	'monitoring_mode = drone_control.monitoring_mode:main',
        ],
    },
)
