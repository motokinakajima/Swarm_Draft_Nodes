import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'peer_detector'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Motoki Nakajima',
    maintainer_email='motoki.nakajima@sps.edu',
    description='Peer detector package',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'detector_node = peer_detector.peer_detector:main',
        ],
    },
)