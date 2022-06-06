from setuptools import setup
import os
from glob import glob
package_name = 'interncoxsys'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name,'launch'), glob('launch/*.launch.py')),

        # Path to the world file
        (os.path.join('share', package_name,'world/'), glob('./world/*')),

        # Path to the mobile robot sdf file
        (os.path.join('share', package_name,'urdf/'), glob('./urdf/*')),

        # Path to the world file (i.e. warehouse + global environment)
        (os.path.join('share', package_name,'urdf/'), glob('./world/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='natta',
    maintainer_email='natta@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'spawn_turtle = interncoxsys.spawn_turtle:main',
        ],
    },
)
