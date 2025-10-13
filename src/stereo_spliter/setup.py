from setuptools import find_packages, setup
import os
from glob import glob
package_name = 'stereo_spliter'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, ['left.yaml']),
        ('share/' + package_name, ['right.yaml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='anhtuanphamben',
    maintainer_email='phamanhtuanben@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'right_stereo_viewer = stereo_spliter.right_stereo_publisher:main',
            'left_stereo_viewer = stereo_spliter.left_stereo_publisher:main', 
            'left_pub = stereo_spliter.left_camera_manager:main',
            'right_pub = stereo_spliter.right_camera_manager:main'
        ],
    },
)
