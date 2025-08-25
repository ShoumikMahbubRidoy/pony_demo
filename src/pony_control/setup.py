from setuptools import setup
import os
from glob import glob

package_name = 'pony_control'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name, f'{package_name}.utils'],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=[
        'setuptools',
        'python-can>=4.0.0',
        'numpy>=1.20.0',
    ],
    zip_safe=True,
    maintainer='Shoumik_Ridoy',
    maintainer_email='smr14421ridoy@gmail.com',
    description='Quadruped robot control package for Pony robot with AK60-6 motors',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'test_single_motor = pony_control.scripts.test_single_motor:main',
            'test_single_leg = pony_control.scripts.test_single_leg:main',
            'quadruped_control = pony_control.scripts.run_quadruped:main',
        ],
    },
)