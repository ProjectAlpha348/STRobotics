from setuptools import setup

package_name = 'robot_head_eyes'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='alpha',
    maintainer_email='alpha@todo.todo',
    description='Eyes node for robot_head (Tommy) using TCA9548A + OLED',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'eyes_node = robot_head_eyes.eyes_node:main',
        ],
    },
)
