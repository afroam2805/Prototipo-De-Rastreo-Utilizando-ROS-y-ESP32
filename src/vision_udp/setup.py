from setuptools import find_packages, setup

package_name = 'vision_udp'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
    ('share/ament_index/resource_index/packages',
        ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
    ('share/' + package_name + '/launch', ['launch/vision_udp.launch.py']),
    ('share/' + package_name + '/rviz', ['rviz/vision_udp.rviz']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='proyecto1',
    maintainer_email='proyecto1@todo.todo',
    description='UDP receiver for vision data and ROS2 topic publishing',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'udp_receiver = vision_udp.udp_receiver:main',
        ],
    },
)
