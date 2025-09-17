from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'viethen_node'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.launch.py'))),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.yaml'))),
        (os.path.join('share', package_name, 'models'), glob(os.path.join('models', '*'))),
    ],
    install_requires=['setuptools', 'moveit_msgs'],
    zip_safe=True,
    maintainer='Abhirup Das',
    maintainer_email='abhirup.das@rwth-aachen.de',
    description='The main node to handle everthing Viethen related',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'viethen_node = viethen_node.viethen_node:main',
            # 'webots_camera_node = viethen_node.webots_camera_node:main',
            # 'scan_coordinator = viethen_node.scan_coordinator:main',
        ],
    },
)