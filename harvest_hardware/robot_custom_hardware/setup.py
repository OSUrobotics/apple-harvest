from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'robot_custom_hardware'

def only_files(paths):
    return [p for p in paths if os.path.isfile(p)]

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, "urdf"), glob('urdf/*')),
        (os.path.join('share', package_name, 'meshes', "base_mounts"), glob(os.path.join('meshes/base_mounts/', '*'))),
        (os.path.join('share', package_name, 'meshes', "end_effectors"), glob(os.path.join('meshes/end_effectors/', '*'))),
        (os.path.join('share', package_name, 'config'), only_files(glob('config/*'))),
        (os.path.join('share', package_name, 'config', 'ur5e'), glob('config/ur5e/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='keegan',
    maintainer_email='keegan.nave@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={"test": ['pytest']},
    entry_points={
        'console_scripts': [
            'gripper_palm_camera = robot_custom_hardware.gripper_palm_camera:main',
        ],
    },
)
