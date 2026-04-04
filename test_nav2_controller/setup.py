from setuptools import setup
from glob import glob
import os

package_name = 'test_nav2_controller'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        # ament index so ros2 can find the package
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        # package.xml
        ('share/' + package_name, ['package.xml']),
        # install  paths (JSONs) into share/<pkg>/paths/
        ('share/' + package_name + '/paths', glob('paths/*.json')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Carlos Hernandez',
    maintainer_email='carlos.salva.hdez@outlook.com',
    description='Stuff for quick testing of Nav2 controller',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # action client to send FollowPath goals:
            'send_follow_path_map = test_nav2_controller.send_follow_path_map:main',
        ],
    },
)
