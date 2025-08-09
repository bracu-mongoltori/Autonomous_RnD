from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'ply_publisher'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kage',
    maintainer_email='kage@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        	'ply_publisher = ply_publisher.ply_pub:main',
            'pcd_publisher = ply_publisher.pcd_publisher:main',
            'costmap_pub = ply_publisher.costmap_pub:main',
            'slope_based_costmap = ply_publisher.slope_based_costmap:main',
        ],
    },
)
