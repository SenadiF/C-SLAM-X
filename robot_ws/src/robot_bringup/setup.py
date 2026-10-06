from setuptools import find_packages, setup

package_name = 'robot_bringup'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),


    ('share/' + package_name + '/launch',
        ['launch/robot.launch.py']),
('share/' + package_name + '/launch',
    ['launch/static_tf.launch.py']),
    ('share/' + package_name + '/launch',
        ['launch/baseline_hw.launch.py']),
    ('share/' + package_name + '/launch',
        ['launch/single_robot_hw.launch.py']),
    ('share/' + package_name + '/rviz',
        ['rviz/single_robot.rviz']),
    ('share/' + package_name + '/urdf',
        ['urdf/cslam_robot.urdf.xacro']),
    ('share/' + package_name + '/config',
        ['config/map_merge_known.yaml']),

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
        ],
    },
)
