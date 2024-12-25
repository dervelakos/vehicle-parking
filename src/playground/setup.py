import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'playground'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*yaml'))),
        (os.path.join('share', package_name, 'maps'), glob(os.path.join('maps', '*.*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='karachatzis',
    maintainer_email='karachatzis.paraskeuas@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'hello = playground.hello:main'
        ],
    },
)
