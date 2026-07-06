from setuptools import find_packages, setup

package_name = 'item_vision'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hye11',
    maintainer_email='hye11@example.com',
    description='RGB-D color and shape perception for factory items',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'vision_node = item_vision.vision_node:main',
        ],
    },
)
