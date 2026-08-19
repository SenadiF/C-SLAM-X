from setuptools import find_packages, setup

package_name = 'my_navigation'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/single_robot_nav.launch.py']),
        ('share/' + package_name + '/launch', ['launch/multi_robot_nav.launch.py']),
        ('share/' + package_name + '/launch', ['launch/map_merge.launch.py']),
        ('share/' + package_name + '/config', ['config/map_merge_params.yaml']),
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
             'frontier_explorer_node = my_navigation.frontier1:main',
              'astar_node = my_navigation.astar:main ',
              'pure_pursuit_node = my_navigation.pure_pursuit_node:main',
              'cmd_vel_relay_node = my_navigation.cmd_vel_relay:main',
              
                ],
    },
)
