from glob import glob
from setuptools import setup

package_name = 'dwa_control'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Giovanny Bano Pena',
    maintainer_email='gio@espol.edu.ec',
    description='Seguimiento de trayectorias con Dynamic Window Approach en AutoDRIVE',
    license='MIT',
    entry_points={
        'console_scripts': [
            'dwa_controller = dwa_control.dwa_controller:main',
        ],
    },
)
