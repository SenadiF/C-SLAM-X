from setuptools import find_packages, setup

package_name = 'auto_nav'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/custom_explore.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='senadi',
    maintainer_email='sdfernando.70@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': ['frontier_explorer = auto_nav.frontier_explorer:main',
        'astar_planner = auto_nav.astar_planner:main',
        'pure_pursuit = auto_nav.pure_pursuit:main',
        'robot_controller = auto_nav.robot_controller:main',
        'robot_coordinator = auto_nav.robot_coordinator:main',
        'simple_controller = auto_nav.simple_controller:main',
        ],
    },
)
