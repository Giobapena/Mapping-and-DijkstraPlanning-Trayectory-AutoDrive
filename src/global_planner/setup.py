from setuptools import setup
from glob import glob

package_name = 'global_planner'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*')),
        ('share/' + package_name + '/rviz', glob('rviz/*.rviz')),
        ('share/' + package_name + '/maps', glob('maps/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Giovanny Andres Bano Pena',
    maintainer_email='giobapena@espol.edu.ec',
    description='Dijkstra + B-Spline sobre el mapa SLAM de AutoDRIVE F1TENTH',
    license='MIT',
    entry_points={
        'console_scripts': [
            'generate_trajectory   = global_planner.generate_trajectory:main',
            'smooth_trajectory     = global_planner.smooth_trajectory:main',
            'generate_gif          = global_planner.generate_gif:main',
            'global_path_publisher = global_planner.global_path_publisher:main',
        ],
    },
)
