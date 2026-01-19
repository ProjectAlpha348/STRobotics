from setuptools import setup
import os
from glob import glob

package_name = 'argo_eyes'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name, f"{package_name}.drivers"],
    data_files=[
        ('share/ament_index/resource_index/packages', [f"resource/{package_name}"]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools', 'pyyaml', 'luma.oled', 'smbus2', 'Pillow'],
    zip_safe=True,
    maintainer='alpha',
    maintainer_email='alpha@todo.todo',
    description='ARGO eyes state machine + rendering scheduler',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'eyes_node = argo_eyes.eyes_node:main',
        ],
    },
)
