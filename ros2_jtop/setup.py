from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'ros2_jtop'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'rqt_gui_setting'), glob(os.path.join('rqt_gui_setting', '*.perspective'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='taitai',
    maintainer_email='taiki11250628@gmail.com',
    description='ROS2 package to monitor jetson resources',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'jtop = ros2_jtop.jtop:main'
        ],
    },
)
