from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'harvest_hardware_description'

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
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*')),
        (os.path.join('share', package_name, 'meshes', 'base_mounts'), glob('meshes/base_mounts/*')),
        (os.path.join('share', package_name, 'meshes', 'end_effectors'), glob('meshes/end_effectors/*')),
        (os.path.join('share', package_name, 'config'), only_files(glob('config/*'))),
        (os.path.join('share', package_name, 'rviz'), only_files(glob('rviz/*'))),
        (os.path.join('share', package_name, 'scripts'), only_files(glob('scripts/*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Marcus Rosette',
    maintainer_email='rosettem@oregonstate.edu',
    description='URDF/xacro description package for harvest hardware (UR5e + gripper/cameras).',
    license='BSD-3-Clause',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gripper_palm_camera = harvest_hardware_description.gripper_palm_camera:main',
            'tcp_pose_relay = harvest_hardware_description.tcp_pose_relay:main',
        ],
    },
)
