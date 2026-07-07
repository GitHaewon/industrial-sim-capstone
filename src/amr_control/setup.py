from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'amr_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (
            os.path.join('share', package_name, 'config'),
            glob('config/*'),
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hye11',
    maintainer_email='hye11@example.com',
    description='Industrial simulation package',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'amr_controller = amr_control.amr_controller:main',
            (
                'dynamic_obstacle_demo = '
                'amr_control.dynamic_obstacle_demo:main'
            ),
        ],
    },
)
