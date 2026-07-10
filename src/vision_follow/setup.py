import os
from glob import glob
from setuptools import setup, find_packages

package_name = 'vision_follow'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', [f'resource/{package_name}']),
        (f'share/{package_name}', ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'params'), glob('params/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Omokai',
    maintainer_email='you@example.com',
    description='Part 4: vision target detect, operator alert, and Nav2-based follow.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'vision_follow_node = vision_follow.vision_follow_node:main',
        ],
    },
)
