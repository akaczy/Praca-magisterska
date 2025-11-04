from setuptools import setup
from glob import glob
import os
package_name = 'robot_symulacja'
data_files = []
data_files.append(('share/ament_index/resource_index/packages', ['resource/' + package_name]))
data_files.append(('share/' + package_name + '/launch', ['launch/symulacja.py']))
data_files.append(('share/' + package_name + '/resource', ['resource/rosbot_controllers.yaml']))
data_files.append(('share/' + package_name + '/resource', ['resource/ekf.yaml']))
data_files.append(('share/' + package_name + '/resource', ['resource/laser_filter.yaml']))
data_files.append(('share/' + package_name + '/worlds', ['worlds/rosbot.wbt']))
data_files.append(('share/' + package_name + '/worlds/meshes', ['worlds/meshes/mesh_world.dae']))
data_files.append(('share/' + package_name + '/protos/icons', glob('protos/icons/*')))
data_files.append(('share/' + package_name + '/protos', glob('protos/*.proto')))
data_files.append(('share/' + package_name, ['package.xml']))

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='----',
    maintainer_email='',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
