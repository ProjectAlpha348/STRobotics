from setuptools import setup, find_packages
import os


package_name = 'robot_head'


def collect_data_files(src_dir: str, install_base: str):
    """
    Colleziona ricorsivamente TUTTI i file sotto src_dir e li installa mantenendo
    la struttura relativa sotto install_base.
    Ritorna una lista di tuple (dest_dir, [files...]) per data_files.
    """
    data_files = []
    for root, _, files in os.walk(src_dir):
        if not files:
            continue
        rel_dir = os.path.relpath(root, src_dir)  # '.' oppure sottocartella
        dest_dir = os.path.join(install_base, rel_dir) if rel_dir != '.' else install_base
        file_list = [os.path.join(root, f) for f in files]
        data_files.append((dest_dir, file_list))
    return data_files


# Base install nello share del package
share_dir = os.path.join('share', package_name)

data_files = [
    (os.path.join('share', 'ament_index', 'resource_index', 'packages'), [os.path.join('resource', package_name)]),
    (share_dir, ['package.xml']),
]

# Installa tutti i launch file
launch_src = 'launch'
if os.path.isdir(launch_src):
    data_files += collect_data_files(launch_src, os.path.join(share_dir, 'launch'))

# Installa TUTTO models/** ricorsivamente
models_src = 'models'
if os.path.isdir(models_src):
    data_files += collect_data_files(models_src, os.path.join(share_dir, 'models'))


setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='alpha',
    maintainer_email='alpha@todo.todo',
    description='Robot head package (vision, voice, orchestrator)',
    license='TODO',
    entry_points={
        'console_scripts': [
            'voice_node = robot_head.voice_node:main',
            'vision_node = robot_head.vision_node:main',
            'speak_node = robot_head.speak_node:main',
            'head_orchestrator = robot_head.head_orchestrator:main',
        ],
    },
)

