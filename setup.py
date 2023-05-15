import os

from glob import glob
from setuptools import setup

package_name = 'img_to_bag_py'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='John Dallas Cast',
    maintainer_email='john.d.cast@gmail.com',
    description='Convert directory of images to bag file',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'a_mode_to_bag = img_to_bag_py.a_mode_to_bag:main',
            'a_mode_pub = img_to_bag_py.a_mode_pub:main',
            'img_viewer = img_to_bag_py.img_viewer:main',
            'img_to_bag = img_to_bag_py.img_to_bag:main',
        ],
    },
)
