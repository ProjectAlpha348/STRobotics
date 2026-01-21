from setuptools import setup

package_name = 'speak'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'piper-tts'],
    zip_safe=True,
    maintainer='STRobotics',
    maintainer_email='info@strobotics.local',
    description='ROS2 Text To Speech node',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'speak = speak.speak_node:main',
        ],
    },
)
