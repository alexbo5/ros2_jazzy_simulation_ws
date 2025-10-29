import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'dual_ur_simulation'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Launch-Dateien
        (os.path.join('share', package_name, 'launch'), 
            glob('launch/*.launch.py')),
        # Config-Dateien
        (os.path.join('share', package_name, 'config'), 
            glob('config/*.yaml')),
        # RViz-Konfigurationen
        (os.path.join('share', package_name, 'rviz'), 
            glob('rviz/*.rviz')),
        # URDF-Dateien
        (os.path.join('share', package_name, 'urdf'), 
            glob('urdf/*')),
        # World-Dateien
        (os.path.join('share', package_name, 'world'), 
            glob('world/*.world')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='alexander-born01@web.de',
    description='TODO: Package description',
    license='Apache-2.0',
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
