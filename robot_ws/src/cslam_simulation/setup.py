from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'cslam_simulation'

setup(
    name=package_name,
    version='0.0.0',

    packages=find_packages(exclude=['test']),

    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name]
        ),

        (
            'share/' + package_name,
            ['package.xml']
        ),

        (
            os.path.join('share', package_name, 'launch'),
            glob('launch/*.py')
        ),
        (
    os.path.join('share', package_name, 'models', 'turtlebot3_robot1'),
    ['models/turtlebot3_robot1/model.sdf']
),

(
    os.path.join('share', package_name, 'models', 'turtlebot3_robot2'),
    ['models/turtlebot3_robot2/model.sdf']
),

        (
            os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')
        ),
    ],

    install_requires=['setuptools'],
    zip_safe=True,

    maintainer='senadi',
    maintainer_email='sdfernando.70@gmail.com',

    description='C-SLAM-X simulation',
    license='TODO: License declaration',

    tests_require=['pytest'],

    entry_points={
        'console_scripts': [],
    },
)