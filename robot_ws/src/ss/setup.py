from setuptools import find_packages, setup

package_name = 'ss'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='senadi',
    maintainer_email='sdfernando.70@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
    'console_scripts': [
        'multirobot_slam_node = ss.multirobot_slam:main',
        'robot_controller_node = ss.robot_controller:main',
        'robot_coordinator_node = ss.robot_coordinator:main',
    ],
},
)
