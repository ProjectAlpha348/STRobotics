from setuptools import setup
import os
from glob import glob

package_name = 'vision'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/models', glob('models/*.onnx')),
        ('share/' + package_name + '/faces_db', glob('faces_db/*.npz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Umberto Di Tullio',
    maintainer_email='alpha348@gmail.com',
    description='Face identification (MediaPipe + ArcFace ONNX) for ROS 2',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
        'face_identify = vision.face_identify_node:main'
        ],
    },
)
