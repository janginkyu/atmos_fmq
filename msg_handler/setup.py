from setuptools import setup, find_packages
from glob import glob
import os

package_name = 'msg_handler'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        # ('share/ament_index/resource_index/packages',
        #     ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob(os.path.join('msg_handler/launch', '*launch.[pxy][yma]*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Inkyu Jang',
    maintainer_email='janginkyu.larr@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'robot = msg_handler.robot:main',
            'controller = msg_handler.controller:main',
            'delay_simulator = msg_handler.delay_simulator:main',
            'delay_compensation_ctrl = msg_handler.delay_compensation_ctrl:main',
            'control_feeder = msg_handler.control_feeder:main',
        ],
    },
)
