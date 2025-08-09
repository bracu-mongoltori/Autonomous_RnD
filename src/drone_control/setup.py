from setuptools import find_packages, setup

package_name = 'drone_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/drone_control.launch.py']),
        ('share/' + package_name + '/config', ['config/drone_config.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kage',
    maintainer_email='hironmoy.roy.rudra@g.bracu.ac.bd',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'drone_controller = drone_control.drone_controller:main',
            'drone_test_client = drone_control.drone_test_client:main',
            'joystick_interface = drone_control.joystick_interface:main',
        ],
    },
)
