from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'ur5e_custom_hardware'

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
        (os.path.join('share', package_name, 'meshes'), glob(os.path.join('meshes', '*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='keegan',
    maintainer_email='keegan.nave@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gripper_palm_camera = ur5e_custom_hardware.gripper_palm_camera:main',
        ],
    },
)
