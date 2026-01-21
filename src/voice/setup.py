from setuptools import setup
from glob import glob
import os

package_name = 'voice'

def package_files(directory):
    paths = []
    for (path, directories, filenames) in os.walk(directory):
        for filename in filenames:
            full_path = os.path.join(path, filename)
            paths.append(full_path)
    return paths

model_files = package_files('models')

data_files = [
    ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
]

# Installa i modelli in share/voice/models/...
for f in model_files:
    install_dir = os.path.join('share', package_name, os.path.dirname(f))
    data_files.append((install_dir, [f]))

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Umberto Di Tullio',
    maintainer_email='',
    description='Vosk STT node that publishes final recognized phrases on /voice',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'recognize = voice.recognize:main',
        ],
    },
)
