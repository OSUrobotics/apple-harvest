from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'harvest_vision'

# List of prosser data directories
prosser_directories = ['prosser_a', 'prosser_b', 'prosser_c', 'prosser_d', 'prosser_e', 'prosser_f', 'prosser_g', 'prosser_h', 'prosser_i', 'prosser_j']

# Generate data mappings for prosser directories
prosser_data_files = [
    (os.path.join('share', package_name, f'data/{directory}'), glob(os.path.join('data', directory, '*')))
    for directory in prosser_directories
]

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'yolo_networks'), glob(os.path.join('yolo_networks', '*'))),
    ] + prosser_data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Keegan Nave',
    maintainer_email='navek@oregonstate.edu',
    description='Vision components for harvest trials',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'apple_prediction = harvest_vision.apple_prediction:main',
            'apple_prediction_presaved_images = harvest_vision.apple_prediction_presaved_images:main',
            'voxelize_scan = harvest_vision.voxelize_scan:main',
        ],
    },
)
