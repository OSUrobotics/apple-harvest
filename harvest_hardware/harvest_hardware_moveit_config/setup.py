from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'harvest_hardware_moveit_config'

def only_files(paths):
    return [p for p in paths if os.path.isfile(p)]

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', [f'resource/{package_name}']),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*launch.[pxy][yma]*')),
        (os.path.join('share', package_name, 'config'), only_files(glob('config/*.yaml'))),
        (os.path.join('share', package_name, 'srdf'), only_files(glob('srdf/*.srdf*'))),
        (os.path.join('share', package_name, 'rviz'), only_files(glob('rviz/*.rviz'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Marcus Rosette',
    maintainer_email='rosettem@oregonstate.edu',
    description='MoveIt configuration package for the harvest hardware (UR5e).',
    license='BSD-3-Clause',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # usually none needed here; add tools if you create any, e.g.:
            # 'dump_groups = harvest_hardware_moveit_config.tools:dump_groups',
        ],
    },
)
