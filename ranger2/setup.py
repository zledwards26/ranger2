from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'ranger2'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

        
        # Install launch files
        (
            os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')
        ),

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ocecengineering',
    maintainer_email='zledwards26@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'look_for_gate = ranger2.LookForGate:main',
            'go_to_gate = ranger2.GoToGate:main',
            'cube_kill_switch = ranger2.cube_kill_switch:main',
            'record = ranger2.Record:main',
            'odom_to_vision_pose = ranger2.odom_to_vision_pose:main'
        ],
    },
)
